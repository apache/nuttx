/****************************************************************************
 * arch/arm/src/sama5/sam_adc.c
 *
 *   Copyright (C) 2013, 2014, 2017-2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code.
 *
 * The Atmel sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 *   Copyright (c) 2012, Atmel Corporation
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
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/semaphore.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "hardware/sam_adc.h"
#include "hardware/sam_pmc.h"
#include "hardware/sam_pinmap.h"

#include "sam_periphclks.h"
#include "sam_memories.h"
#include "sam_pio.h"
#include "sam_dmac.h"
#include "sam_tc.h"
#include "sam_tsd.h"
#include "sam_adc.h"

#if defined(CONFIG_SAMA5_ADC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Count the number of channels in use */

#define SAMA5_CHAN0_INUSE    0
#define SAMA5_CHAN1_INUSE    0
#define SAMA5_CHAN2_INUSE    0
#define SAMA5_CHAN3_INUSE    0
#define SAMA5_CHAN4_INUSE    0
#define SAMA5_CHAN5_INUSE    0
#define SAMA5_CHAN6_INUSE    0
#define SAMA5_CHAN7_INUSE    0
#define SAMA5_CHAN8_INUSE    0
#define SAMA5_CHAN9_INUSE    0
#define SAMA5_CHAN10_INUSE   0
#define SAMA5_CHAN11_INUSE   0

#ifdef CONFIG_SAMA5_ADC_CHAN0
#  undef  SAMA5_CHAN0_INUSE
#  define SAMA5_CHAN0_INUSE  1
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN1
#  undef  SAMA5_CHAN1_INUSE
#  define SAMA5_CHAN1_INUSE  1
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN2
#  undef  SAMA5_CHAN2_INUSE
#  define SAMA5_CHAN2_INUSE  1
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN3
#  undef  SAMA5_CHAN3_INUSE
#  define SAMA5_CHAN3_INUSE  1
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN4
#  undef  SAMA5_CHAN4_INUSE
#  define SAMA5_CHAN4_INUSE  1
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN5
#  undef  SAMA5_CHAN5_INUSE
#  define SAMA5_CHAN5_INUSE  1
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN6
#  undef  SAMA5_CHAN6_INUSE
#  define SAMA5_CHAN6_INUSE  1
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN7
#  undef  SAMA5_CHAN7_INUSE
#  define SAMA5_CHAN7_INUSE  1
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN8
#  undef  SAMA5_CHAN8_INUSE
#  define SAMA5_CHAN8_INUSE  1
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN9
#  undef  SAMA5_CHAN9_INUSE
#  define SAMA5_CHAN9_INUSE  1
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN10
#  undef  SAMA5_CHAN10_INUSE
#  define SAMA5_CHAN10_INUSE 1
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN11
#  undef  SAMA5_CHAN11_INUSE
#  define SAMA5_CHAN11_INUSE 1
#endif

#define SAMA5_NCHANNELS \
  (SAMA5_CHAN0_INUSE + SAMA5_CHAN1_INUSE  + SAMA5_CHAN2_INUSE  + \
   SAMA5_CHAN3_INUSE + SAMA5_CHAN4_INUSE  + SAMA5_CHAN5_INUSE  + \
   SAMA5_CHAN6_INUSE + SAMA5_CHAN7_INUSE  + SAMA5_CHAN8_INUSE  + \
   SAMA5_CHAN9_INUSE + SAMA5_CHAN10_INUSE + SAMA5_CHAN11_INUSE)

/* Get the set of channel interrupts to enable */

#define SAMA5_CHAN0_ENABLE    0
#define SAMA5_CHAN1_ENABLE    0
#define SAMA5_CHAN2_ENABLE    0
#define SAMA5_CHAN3_ENABLE    0
#define SAMA5_CHAN4_ENABLE    0
#define SAMA5_CHAN5_ENABLE    0
#define SAMA5_CHAN6_ENABLE    0
#define SAMA5_CHAN7_ENABLE    0
#define SAMA5_CHAN8_ENABLE    0
#define SAMA5_CHAN9_ENABLE    0
#define SAMA5_CHAN10_ENABLE   0
#define SAMA5_CHAN11_ENABLE   0

#if defined(CONFIG_SAMA5_ADC_CHAN0)
#  undef  SAMA5_CHAN0_ENABLE
#  define SAMA5_CHAN0_ENABLE  ADC_INT_EOC0
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN1)
#  undef  SAMA5_CHAN1_ENABLE
#  define SAMA5_CHAN1_ENABLE  ADC_INT_EOC1
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN2)
#  undef  SAMA5_CHAN2_ENABLE
#  define SAMA5_CHAN2_ENABLE  ADC_INT_EOC2
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN3)
#  undef  SAMA5_CHAN3_ENABLE
#  define SAMA5_CHAN3_ENABLE  ADC_INT_EOC3
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN4)
#  undef  SAMA5_CHAN4_ENABLE
#  define SAMA5_CHAN4_ENABLE  ADC_INT_EOC4
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN5)
#  undef  SAMA5_CHAN5_ENABLE
#  define SAMA5_CHAN5_ENABLE  ADC_INT_EOC5
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN6)
#  undef  SAMA5_CHAN6_ENABLE
#  define SAMA5_CHAN6_ENABLE  ADC_INT_EOC6
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN7)
#  undef  SAMA5_CHAN7_ENABLE
#  define SAMA5_CHAN7_ENABLE  ADC_INT_EOC7
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN8)
#  undef  SAMA5_CHAN8_ENABLE
#  define SAMA5_CHAN8_ENABLE  ADC_INT_EOC8
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN9)
#  undef  SAMA5_CHAN9_ENABLE
#  define SAMA5_CHAN9_ENABLE  ADC_INT_EOC9
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN10)
#  undef  SAMA5_CHAN10_ENABLE
#  define SAMA5_CHAN10_ENABLE  ADC_INT_EOC10
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN11)
#  undef  SAMA5_CHAN11_ENABLE
#  define SAMA5_CHAN11_ENABLE  ADC_INT_EOC11
#endif

#define SAMA5_CHAN_ENABLE \
  (SAMA5_CHAN0_ENABLE | SAMA5_CHAN1_ENABLE  | SAMA5_CHAN2_ENABLE  | \
   SAMA5_CHAN3_ENABLE | SAMA5_CHAN4_ENABLE  | SAMA5_CHAN5_ENABLE  | \
   SAMA5_CHAN6_ENABLE | SAMA5_CHAN7_ENABLE  | SAMA5_CHAN8_ENABLE  | \
   SAMA5_CHAN9_ENABLE | SAMA5_CHAN10_ENABLE | SAMA5_CHAN11_ENABLE)

/* If we are supporting the analog change feature, then sure that there
 * is a gain setting for each enabled channel.
 *
 * Valid gain settings are {0, 1, 2, 3} which may be interpreted as
 * either {1, 1, 2, 4} if the DIFFx bit in COR register is zero or as
 * {0.5, 1, 2, 2} if the DIFFx bit is set.
 */

#ifdef CONFIG_SAMA5_ADC_ANARCH
#  undef CONFIG_SAMA5_ADC_GAIN
#  if defined(CONFIG_SAMA5_ADC_CHAN0) && !defined(CONFIG_SAMA5_ADC_GAIN0)
#    define CONFIG_SAMA5_ADC_GAIN0 1
#  endif
#  if defined(CONFIG_SAMA5_ADC_CHAN1) && !defined(CONFIG_SAMA5_ADC_GAIN1)
#    define CONFIG_SAMA5_ADC_GAIN1 1
#  endif
#  if defined(CONFIG_SAMA5_ADC_CHAN2) && !defined(CONFIG_SAMA5_ADC_GAIN2)
#    define CONFIG_SAMA5_ADC_GAIN2 1
#  endif
#  if defined(CONFIG_SAMA5_ADC_CHAN3) && !defined(CONFIG_SAMA5_ADC_GAIN3)
#    define CONFIG_SAMA5_ADC_GAIN3 1
#  endif
#  if defined(CONFIG_SAMA5_ADC_CHAN4) && !defined(CONFIG_SAMA5_ADC_GAIN4)
#    define CONFIG_SAMA5_ADC_GAIN4 1
#  endif
#  if defined(CONFIG_SAMA5_ADC_CHAN5) && !defined(CONFIG_SAMA5_ADC_GAIN5)
#    define CONFIG_SAMA5_ADC_GAIN5 1
#  endif
#  if defined(CONFIG_SAMA5_ADC_CHAN6) && !defined(CONFIG_SAMA5_ADC_GAIN6)
#    define CONFIG_SAMA5_ADC_GAIN6 1
#  endif
#  if defined(CONFIG_SAMA5_ADC_CHAN7) && !defined(CONFIG_SAMA5_ADC_GAIN7)
#    define CONFIG_SAMA5_ADC_GAIN7 1
#  endif
#  if defined(CONFIG_SAMA5_ADC_CHAN8) && !defined(CONFIG_SAMA5_ADC_GAIN8)
#    define CONFIG_SAMA5_ADC_GAIN8 1
#  endif
#  if defined(CONFIG_SAMA5_ADC_CHAN9) && !defined(CONFIG_SAMA5_ADC_GAIN9)
#    define CONFIG_SAMA5_ADC_GAIN9 1
#  endif
#  if defined(CONFIG_SAMA5_ADC_CHAN10) && !defined(CONFIG_SAMA5_ADC_GAIN10)
#    define CONFIG_SAMA5_ADC_GAIN10 1
#  endif
#  if defined(CONFIG_SAMA5_ADC_CHAN11) && !defined(CONFIG_SAMA5_ADC_GAIN11)
#    define CONFIG_SAMA5_ADC_GAIN11 1
#  endif

/* Otherwise, make sure the single global gain value is defined */

#else
#  ifndef CONFIG_SAMA5_ADC_GAIN
#    define CONFIG_SAMA5_ADC_GAIN 1
#  endif
#  undef CONFIG_SAMA5_ADC_GAIN0
#  undef CONFIG_SAMA5_ADC_GAIN1
#  undef CONFIG_SAMA5_ADC_GAIN2
#  undef CONFIG_SAMA5_ADC_GAIN3
#  undef CONFIG_SAMA5_ADC_GAIN4
#  undef CONFIG_SAMA5_ADC_GAIN5
#  undef CONFIG_SAMA5_ADC_GAIN6
#  undef CONFIG_SAMA5_ADC_GAIN7
#  undef CONFIG_SAMA5_ADC_GAIN8
#  undef CONFIG_SAMA5_ADC_GAIN9
#  undef CONFIG_SAMA5_ADC_GAIN10
#  undef CONFIG_SAMA5_ADC_GAIN11
#endif

/* Check timer configuration */

#if defined(CONFIG_SAMA5_ADC_TIOATRIG) && !defined(CONFIG_SAMA5_TC0)
#  error CONFIG_SAMA5_ADC_TIOATRIG requires CONFIG_SAMA5_TC0
#endif

/* Determine the set channels that are available.  Not all channels will be
 * available if the touch screen is enabled
 */

#ifdef CONFIG_SAMA5_TSD
#  ifdef CONFIG_SAMA5_TSD_5WIRE
#    define SAMA5_ADC_CHALL  (ADC_CHALL & ~TSD_5WIRE_ALL)
#  else
#    define SAMA5_ADC_CHALL  (ADC_CHALL & ~TSD_4WIRE_ALL)
#  endif
#else
#  define SAMA5_ADC_CHALL    ADC_CHALL
#endif

/* DMA configuration flags */

#ifdef CONFIG_SAMA5_ADC_DMA
#  define DMA_FLAGS \
     DMACH_FLAG_FIFOCFG_LARGEST | \
     DMACH_FLAG_PERIPHPID(SAM_IRQ_ADC) | DMACH_FLAG_PERIPHAHB_AHB_IF2 | \
     DMACH_FLAG_PERIPHH2SEL | DMACH_FLAG_PERIPHISPERIPH |  \
     DMACH_FLAG_PERIPHWIDTH_16BITS | DMACH_FLAG_PERIPHCHUNKSIZE_1 | \
     DMACH_FLAG_MEMPID_MAX | DMACH_FLAG_MEMAHB_AHB_IF0 | \
     DMACH_FLAG_MEMWIDTH_16BITS | DMACH_FLAG_MEMINCREMENT | \
     DMACH_FLAG_MEMCHUNKSIZE_1 | DMACH_FLAG_MEMBURST_4)
#endif

/* Pick an unused channel number */

#if !defined(CONFIG_SAMA5_ADC_CHAN0)
#  define SAMA5_ADC_UNUSED 0
#elif !defined(CONFIG_SAMA5_ADC_CHAN1)
#  define SAMA5_ADC_UNUSED 1
#elif !defined(CONFIG_SAMA5_ADC_CHAN2)
#  define SAMA5_ADC_UNUSED 2
#elif !defined(CONFIG_SAMA5_ADC_CHAN3)
#  define SAMA5_ADC_UNUSED 3
#elif !defined(CONFIG_SAMA5_ADC_CHAN4)
#  define SAMA5_ADC_UNUSED 4
#elif !defined(CONFIG_SAMA5_ADC_CHAN5)
#  define SAMA5_ADC_UNUSED 5
#elif !defined(CONFIG_SAMA5_ADC_CHAN6)
#  define SAMA5_ADC_UNUSED 6
#elif !defined(CONFIG_SAMA5_ADC_CHAN7)
#  define SAMA5_ADC_UNUSED 7
#elif !defined(CONFIG_SAMA5_ADC_CHAN8)
#  define SAMA5_ADC_UNUSED 8
#elif !defined(CONFIG_SAMA5_ADC_CHAN9)
#  define SAMA5_ADC_UNUSED 9
#elif !defined(CONFIG_SAMA5_ADC_CHAN10)
#  define SAMA5_ADC_UNUSED 10
#elif !defined(CONFIG_SAMA5_ADC_CHAN11)
#  define SAMA5_ADC_UNUSED 11
#else
#  undef SAMA5_ADC_UNUSED
#endif

/* Number of DMA samples to collect */

#if !defined(CONFIG_SAMA5_ADC_DMA)
#  undef  CONFIG_SAMA5_ADC_DMASAMPLES
#  define CONFIG_SAMA5_ADC_DMASAMPLES 1
#elif !defined(CONFIG_SAMA5_ADC_DMASAMPLES)
#  error CONFIG_SAMA5_ADC_DMASAMPLES must be defined
#elif CONFIG_SAMA5_ADC_DMASAMPLES < 2
#  warning Values of ONFIG_SAMA5_ADC_DMASAMPLES < 2 are inefficient
#endif

#define SAMA5_ADC_SAMPLES (CONFIG_SAMA5_ADC_DMASAMPLES * SAMA5_NCHANNELS)

#ifndef CONFIG_DEBUG_ANALOG_INFO
#  undef CONFIG_SAMA5_ADC_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the overall state of the ADC */

struct sam_adc_s
{
  FAR const struct adc_callback_s *cb;
  sem_t exclsem;         /* Supports exclusive access to the ADC interface */
  bool initialized;      /* The ADC driver is already initialized */
  uint32_t frequency;    /* ADC clock frequency */

#ifdef SAMA5_ADC_HAVE_CHANNELS
#ifdef CONFIG_SAMA5_ADC_DMA
  volatile bool odd;     /* Odd buffer is in use */
  volatile bool ready;   /* Worker has completed the last set of samples */
  volatile bool enabled; /* DMA data transfer is enabled */
#endif
  struct adc_dev_s *dev; /* A reference to the outer, ADC device container */
  uint32_t pending;      /* Pending EOC events */
  struct work_s work;    /* Supports the interrupt handling "bottom half" */
#ifdef CONFIG_SAMA5_ADC_DMA
  DMA_HANDLE dma;        /* Handle for DMA channel */
#endif
#ifdef CONFIG_SAMA5_ADC_TIOATRIG
  TC_HANDLE tc;          /* Handle for the timer channel */
#endif

  /* DMA sample data buffer */

#ifdef CONFIG_SAMA5_ADC_DMA
  uint16_t evenbuf[SAMA5_ADC_SAMPLES];
  uint16_t oddbuf[SAMA5_ADC_SAMPLES];
#endif
#endif /* SAMA5_ADC_HAVE_CHANNELS */

  /* Debug stuff */

#ifdef CONFIG_SAMA5_ADC_REGDEBUG
  bool wrlast;          /* Last was a write */
  uintptr_t addrlast;   /* Last address */
  uint32_t vallast;     /* Last value */
  int ntimes;           /* Number of times */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_SAMA5_ADC_REGDEBUG
static bool sam_adc_checkreg(struct sam_adc_s *priv, bool wr,
                             uint32_t regval, uintptr_t address);
#endif

/* DMA helper functions */

#ifdef SAMA5_ADC_HAVE_CHANNELS
#ifdef CONFIG_SAMA5_ADC_DMA
static void sam_adc_dmadone(void *arg);
static void sam_adc_dmacallback(DMA_HANDLE handle, void *arg, int result);
static int  sam_adc_dmasetup(struct sam_adc_s *priv, FAR uint8_t *buffer,
                             size_t buflen);
static void sam_adc_dmastart(struct sam_adc_s *priv);
#endif

/* ADC interrupt handling */

static void sam_adc_endconversion(void *arg);
#endif
static int  sam_adc_interrupt(int irq, void *context, FAR void *arg);

/* ADC methods */

#ifdef SAMA5_ADC_HAVE_CHANNELS
static int  sam_adc_bind(FAR struct adc_dev_s *dev,
                         FAR const struct adc_callback_s *callback);
static void sam_adc_reset(struct adc_dev_s *dev);
static int  sam_adc_setup(struct adc_dev_s *dev);
static void sam_adc_shutdown(struct adc_dev_s *dev);
static void sam_adc_rxint(struct adc_dev_s *dev, bool enable);
static int  sam_adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg);

/* Initialization/Configuration */

#ifdef CONFIG_SAMA5_ADC_TIOATRIG
static int  sam_adc_settimer(struct sam_adc_s *priv, uint32_t frequency,
                             int channel);
static void sam_adc_freetimer(struct sam_adc_s *priv);
#endif
static int  sam_adc_trigger(struct sam_adc_s *priv);
static void sam_adc_autocalibrate(struct sam_adc_s *priv);
static void sam_adc_offset(struct sam_adc_s *priv);
static void sam_adc_gain(struct sam_adc_s *priv);
static void sam_adc_analogchange(struct sam_adc_s *priv);
static void sam_adc_sequencer(struct sam_adc_s *priv);
static void sam_adc_channels(struct sam_adc_s *priv);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef SAMA5_ADC_HAVE_CHANNELS
/* ADC lower half device operations */

static const struct adc_ops_s g_adcops =
{
  .ao_bind     = sam_adc_bind,
  .ao_reset    = sam_adc_reset,
  .ao_setup    = sam_adc_setup,
  .ao_shutdown = sam_adc_shutdown,
  .ao_rxint    = sam_adc_rxint,
  .ao_ioctl    = sam_adc_ioctl,
};
#endif

/* ADC internal state */

static struct sam_adc_s g_adcpriv;

/* ADC device instance */

static struct adc_dev_s g_adcdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Register Operations
 ****************************************************************************/

/****************************************************************************
 * Name: sam_adc_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   regval  - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_ADC_REGDEBUG
static bool sam_adc_checkreg(struct sam_adc_s *priv, bool wr,
                             uint32_t regval, uintptr_t address)
{
  if (wr      == priv->wrlast &&   /* Same kind of access? */
      regval  == priv->vallast &&  /* Same value? */
      address == priv->addrlast)   /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      priv->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (priv->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          ainfo("...[Repeats %d times]...\n", priv->ntimes);
        }

      /* Save information about the new access */

      priv->wrlast   = wr;
      priv->vallast  = regval;
      priv->addrlast = address;
      priv->ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

#ifdef SAMA5_ADC_HAVE_CHANNELS

/****************************************************************************
 * DMA Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: sam_adc_dmadone
 *
 * Description:
 *   This function executes on the worker thread.  It is scheduled by
 *   sam_adc_dmacallback at the complete of each DMA sequenece.  There is
 *   and interlock using ping-pong buffers and boolean values to prevent
 *   overrunning the worker thread:
 *
 *     oddbuf[]/evenbuf[] - Ping pong buffers are used.  The DMA collects
 *       data in one buffer while the worker thread processes data in the
 *       other.
 *     odd - If true, then DMA is active in the oddbuf[]; evenbuf[] holds
 *       completed DMA data.
 *     ready - Ping ponging is halted while ready is false;  If data overrun
 *       occurs, then sample data will be lost on one sequence.  The worker
 *       thread sets ready when it has completed processing the last sample
 *       data.
 *
 * Input Parameters:
 *   arg - The ADC private data structure cast to (void *)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_ADC_DMA
static void sam_adc_dmadone(void *arg)
{
  struct sam_adc_s *priv = (struct sam_adc_s *)arg;
  uint16_t *buffer;
  uint16_t *next;
  uint16_t sample;
  int chan;
  int i;

  ainfo("ready=%d enabled=%d\n", priv->enabled, priv->ready);
  DEBUGASSERT(priv != NULL && !priv->ready);

  /* If the DMA transfer is not enabled, just ignore the data (and do not
   * start the next DMA transfer).
   */

  if (priv->enabled)
    {
      /* Toggle to the next buffer.
       *
       *   buffer - The buffer on which the DMA has just completed
       *   next   - The buffer in which to start the next DMA
       */

      if (priv->odd)
        {
          buffer    = priv->oddbuf;
          next      = priv->evenbuf;
          priv->odd = false;
        }
      else
        {
          buffer    = priv->evenbuf;
          next      = priv->oddbuf;
          priv->odd = true;
        }

      /* Restart the DMA conversion as quickly as possible using the next
       * buffer.
       *
       * REVISIT: In the original design, toggling the ping-pong buffers and
       * restarting the DMA was done in the interrupt handler so that the
       * next buffer could be filling while the current buffer is being
       * processed here on the worker thread.  But, unfortunately,
       * sam_adcm_dmasetup() cannot be called from an interrupt handler.
       *
       * A consequence of this is that there is a small window from the time
       * that the last set of samples was taken, the worker thread runs, and
       * the follow logic restarts the DMA in which samples could be lost!
       *
       * Without the interrupt level DMA restart logic, there is not really
       * any good reason to support the ping-poing buffers at all.
       */

      sam_adc_dmasetup(priv, (FAR uint8_t *)next,
                       SAMA5_ADC_SAMPLES * sizeof(uint16_t));

      /* Invalidate the DMA buffer so that we are guaranteed to reload the
       * newly DMAed data from RAM.
       */

      up_invalidate_dcache((uintptr_t)buffer,
                           (uintptr_t)buffer +
                           SAMA5_ADC_SAMPLES * sizeof(uint16_t));

      /* Process each sample */

      for (i = 0; i < SAMA5_ADC_SAMPLES; i++, buffer++)
        {
          /* Get the sample and the channel number */

          chan   = (int)((*buffer & ADC_LCDR_CHANB_MASK) >>
                    ADC_LCDR_CHANB_SHIFT);
          sample = ((*buffer & ADC_LCDR_DATA_MASK) >> ADC_LCDR_DATA_SHIFT);

          /* Verify the upper-half driver has bound its callback functions */

          if (priv->cb != NULL)
            {
              /* Give the sample data to the ADC upper half */

              DEBUGASSERT(priv->cb->au_receive != NULL);
              priv->cb->au_receive(priv->dev, chan, sample);
            }
        }
    }

  /* We are ready to handle the next sample sequence */

  priv->ready = true;
}
#endif

/****************************************************************************
 * Name: sam_adc_dmastart
 *
 * Description:
 *   Initiate DMA sampling.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_ADC_DMA
static void sam_adc_dmastart(struct sam_adc_s *priv)
{
  /* Make sure that the worker is available and that DMA is not disabled */

  if (priv->ready && priv->enabled)
    {
      priv->odd = false;  /* Start with the even buffer */
      sam_adc_dmasetup(priv, (FAR uint8_t *)priv->evenbuf,
                       SAMA5_ADC_SAMPLES * sizeof(uint16_t));
    }
}
#endif

/****************************************************************************
 * Name: sam_adc_dmacallback
 *
 * Description:
 *   Called when one ADC DMA sequence completes.  This function defers
 *   processing of the samples to sam_adc_dmadone which runs on the worker
 *   thread.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_ADC_DMA
static void sam_adc_dmacallback(DMA_HANDLE handle, void *arg, int result)
{
  struct sam_adc_s *priv = (struct sam_adc_s *)arg;
  int ret;

  ainfo("ready=%d enabled=%d\n", priv->enabled, priv->ready);
  DEBUGASSERT(priv->ready);

  /* Check of the bottom half is keeping up with us.
   *
   * ready == false:  Would mean that the worker thready has not ran since
   *   the last DMA callback.
   * enabled == false: Means that the upper half has asked us nicely to stop
   *   transferring DMA data.
   */

  if (priv->ready && priv->enabled)
    {
      /* Verify that the worker is available */

      DEBUGASSERT(priv->work.worker == NULL);

      /* Mark the work as busy and schedule the DMA done processing to
       * occur on the worker thread.
       */

      priv->ready = false;

      ret = work_queue(HPWORK, &priv->work, sam_adc_dmadone, priv, 0);
      if (ret != 0)
        {
          aerr("ERROR: Failed to queue work: %d\n", ret);
        }
    }

  /* REVISIT: There used to be logic here to toggle the ping-pong buffers and
   * to restart the DMA conversion.  This would allow refilling one buffer
   * while the worker processes the other buffer that was just filled. But,
   * unfortunately, sam_adcm_dmasetup() and dma_rxsetup cannot be called
   * from an interrupt handler.
   *
   * A consequence of this is that there is a small window from the time
   * that the last set of samples was taken, the worker thread runs, and the
   * logic on the worker thread restarts the DMA.  Samples trigger during
   * this window will be be lost!
   *
   * Without this logic, there is not really any strong reason to support
   * the ping-poing buffers at all.
   */
}
#endif

/****************************************************************************
 * Name: sam_adc_dmasetup
 *
 * Description:
 *   Setup to perform a read DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For read transfers this may mean
 *   invalidating the data cache.
 *
 * Input Parameters:
 *   priv   - An instance of the ADC device interface
 *   buffer - The memory to DMA from
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_ADC_DMA
static int sam_adc_dmasetup(FAR struct sam_adc_s *priv, FAR uint8_t *buffer,
                            size_t buflen)
{
  uint32_t paddr;
  uint32_t maddr;

  ainfo("buffer=%p buflen=%d\n", buffer, (int)buflen);
  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Physical address of the ADC LCDR register and of the buffer location in
   * RAM.
   */

  paddr = sam_physregaddr(SAM_ADC_LCDR);
  maddr = sam_physramaddr((uintptr_t)buffer);

  /* Configure the RX DMA */

  sam_dmarxsetup(priv->dma, paddr, maddr, buflen);

  /* Start the DMA */

  sam_dmastart(priv->dma, sam_adc_dmacallback, priv);
  return OK;
}
#endif

/****************************************************************************
 * ADC interrupt handling
 ****************************************************************************/

/****************************************************************************
 * Name: sam_adc_endconversion
 *
 * Description:
 *   This function executes on the worker thread.  It is scheduled by
 *   sam_adc_interrupt whenever any enabled end-of-conversion event occurs.
 *   All EOC interrupts are disabled when this function runs.
 *   sam_adc_endconversion will re-enable EOC interrupts when it completes
 *   processing all pending EOC events.
 *
 * Input Parameters:
 *   arg - The ADC private data structure cast to (void *)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_adc_endconversion(void *arg)
{
  struct sam_adc_s *priv = (struct sam_adc_s *)arg;
  uint32_t regval;
  uint32_t pending;
  int chan;
  int ret;

  DEBUGASSERT(priv != NULL);
  ainfo("pending=%08x\n", priv->pending);

  /* Get the set of unmasked, pending ADC interrupts */

  pending = priv->pending;

  /* Get exclusive access to the driver data structure */

  ret = sam_adc_lock(priv);
  if (ret < 0)
    {
      return;
    }

  /* Check for the end of conversion event on each channel */

  for (chan = 0; chan < SAM_ADC_NCHANNELS && pending != 0; chan++)
    {
      uint32_t bit = ADC_INT_EOC(chan);
      if ((pending & bit) != 0)
        {
          /* Read the ADC sample and pass it to the upper half */

          regval = sam_adc_getreg(priv, SAM_ADC_CDR(chan));

          /* Verify the upper-half driver has bound its callback functions */

          if (priv->cb != NULL)
            {
              /* Perform the data received callback */

              DEBUGASSERT(priv->cb->au_receive != NULL);
              priv->cb->au_receive(priv->dev, chan, regval &
                                   ADC_CDR_DATA_MASK);
            }

          pending &= ~bit;
        }
    }

  /* Exit, re-enabling ADC interrupts */

  sam_adc_putreg(priv, SAM_ADC_IER, SAMA5_CHAN_ENABLE);

  /* Release our lock on the ADC structure */

  sam_adc_unlock(priv);
}
#endif /* SAMA5_ADC_HAVE_CHANNELS */

/****************************************************************************
 * Name: sam_adc_interrupt
 *
 * Description:
 *   ADC interrupt handler
 *
 ****************************************************************************/

static int sam_adc_interrupt(int irq, void *context, FAR void *arg)
{
  struct sam_adc_s *priv = &g_adcpriv;
  uint32_t isr;
  uint32_t imr;
  uint32_t pending;

  /* Get the set of unmasked, pending ADC interrupts */

  isr     = sam_adc_getreg(priv, SAM_ADC_ISR);
  imr     = sam_adc_getreg(priv, SAM_ADC_IMR);
  pending = isr & imr;

  /* Handle pending touchscreen interrupts */

#ifdef CONFIG_SAMA5_TSD
  if ((pending & ADC_TSD_ALLINTS) != 0)
    {
      /* Let the touchscreen handle its interrupts.  Pass the pending
       * interrupt set PLUS the pen status bit.
       */

      sam_tsd_interrupt(isr & (imr | ADC_SR_PENS));
      pending &= ~ADC_TSD_ALLINTS;
    }
#endif

#ifdef SAMA5_ADC_HAVE_CHANNELS
  /* Check for end-of-conversion interrupts */

  if ((pending & ADC_INT_EOCALL) != 0)
    {
      int ret;

      /* Disable further end-of-conversion interrupts.  End-of-conversion
       * interrupts will be re-enabled after the worker thread executes.
       */

      sam_adc_putreg(priv, SAM_ADC_IDR, ADC_INT_EOCALL);

      /* Save the set of pending interrupts for the bottom half (in case any
       * were cleared by reading the ISR).
       */

      priv->pending = pending;

      /* Transfer processing to the worker thread.  Since end-of-conversion
       * interrupts are disabled while the work is pending, no special action
       * should be required to protected the work queue.
       */

      DEBUGASSERT(priv->work.worker == NULL);
      ret = work_queue(HPWORK, &priv->work, sam_adc_endconversion, priv, 0);
      if (ret != 0)
        {
          aerr("ERROR: Failed to queue work: %d\n", ret);
        }

      pending &= ~ADC_INT_EOCALL;
    }
#endif

  /* Make sure that all interrupts were handled */

  DEBUGASSERT(pending == 0);
  UNUSED(priv);  /* Not used in all configurations */
  return OK;
}

#ifdef SAMA5_ADC_HAVE_CHANNELS
/****************************************************************************
 * ADC methods
 ****************************************************************************/

/****************************************************************************
 * Name: sam_adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int sam_adc_bind(FAR struct adc_dev_s *dev,
                        FAR const struct adc_callback_s *callback)
{
  struct sam_adc_s *priv = (struct sam_adc_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: sam_adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before sam_adc_setup() and on error conditions.
 *
 ****************************************************************************/

static void sam_adc_reset(struct adc_dev_s *dev)
{
#ifdef CONFIG_SAMA5_ADC_DMA
  struct sam_adc_s *priv = (struct sam_adc_s *)dev->ad_priv;
#endif
  uint32_t regval;

  ainfo("Resetting..\n");

  /* NOTE: We can't really reset the ADC hardware without losing the
   * touchscreen configuration.
   */

  /* Stop any ongoing DMA */

#ifdef CONFIG_SAMA5_ADC_DMA
  if (priv->dma)
    {
      sam_dmastop(priv->dma);
    }
#endif

  /* Stop an release any timer */

#ifdef CONFIG_SAMA5_ADC_TIOATRIG
  sam_adc_freetimer(priv);
#endif

  /* Disable all EOC interrupts */

  sam_adc_putreg(priv, SAM_ADC_IDR, ADC_INT_EOCALL);

  /* Disable all channels */

  sam_adc_putreg(priv, SAM_ADC_CHDR, SAMA5_ADC_CHALL);

  /* Disable the sequencer and analog change */

  regval  = sam_adc_getreg(priv, SAM_ADC_MR);
  regval &= ~(ADC_MR_USEQ | ADC_MR_ANACH);
  sam_adc_putreg(priv, SAM_ADC_MR, regval);

  /* Reset gain, offset, differential modes */

  sam_adc_putreg(priv, SAM_ADC_CGR, 0);
  sam_adc_putreg(priv, SAM_ADC_COR, 0);

#ifndef CONFIG_SAMA5_ADC_SWTRIG
  /* Select software trigger (i.e., basically no trigger) */

  regval  = sam_adc_getreg(priv, SAM_ADC_MR);
  regval &= ~ADC_MR_TRGSEL_MASK;
  sam_adc_putreg(priv, SAM_ADC_MR, regval);

  regval  = sam_adc_getreg(priv, SAM_ADC_TRGR);
  regval &= ~ADC_TRGR_TRGMOD_MASK;
  regval |= ADC_TRGR_TRGMOD_NOTRIG;
  sam_adc_putreg(priv, SAM_ADC_TRGR, regval);
#endif
}

/****************************************************************************
 * Name: sam_adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts.
 *   Interrupts are all disabled upon return.
 *
 ****************************************************************************/

static int sam_adc_setup(struct adc_dev_s *dev)
{
  struct sam_adc_s *priv = (struct sam_adc_s *)dev->ad_priv;
  uint32_t regval;

  ainfo("Setup\n");

  /* Enable channel number tag.  This bit will force the channel number
   * (CHNB) to be included in the LDCR register content.
   */

  regval  = sam_adc_getreg(priv, SAM_ADC_EMR);
  regval |= ADC_EMR_TAG;
  sam_adc_putreg(priv, SAM_ADC_EMR, regval);

  /* Enable (or disable) the sequencer */

  sam_adc_sequencer(priv);

  /* Enable ADC channels */

  sam_adc_channels(priv);

  /* Enable/disable analog change.  This feature permits different settings
   * per channel.
   */

  sam_adc_analogchange(priv);

  /* Set gain */

  sam_adc_gain(priv);

  /* Set offset and single/differential mode */

  sam_adc_offset(priv);

  /* Perform Auto Calibration */

  sam_adc_autocalibrate(priv);

#ifdef CONFIG_SAMA5_ADC_DMA
  /* Initiate DMA transfers */

  priv->ready   = true;   /* Worker is available */
  priv->enabled = true;   /* Transfers are enabled */

  sam_adc_dmastart(priv);

#else
  /* Enable end-of-conversion interrupts for all enabled channels. */

  sam_adc_putreg(priv, SAM_ADC_IER, SAMA5_CHAN_ENABLE);

#endif

  /* Configure trigger mode and start conversion */

  return sam_adc_trigger(priv);
}

/****************************************************************************
 * Name: sam_adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void sam_adc_shutdown(struct adc_dev_s *dev)
{
#ifdef CONFIG_SAMA5_ADC_DMA
  struct sam_adc_s *priv = (struct sam_adc_s *)dev->ad_priv;
#endif

  ainfo("Shutdown\n");

  /* Reset the ADC peripheral */

  sam_adc_reset(dev);

  /* Disable ADC interrupts at the level of the AIC */

  up_disable_irq(SAM_IRQ_ADC);

  /* Then detach the ADC interrupt handler. */

  irq_detach(SAM_IRQ_ADC);
}

/****************************************************************************
 * Name: sam_adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void sam_adc_rxint(struct adc_dev_s *dev, bool enable)
{
#ifdef CONFIG_SAMA5_ADC_DMA
  struct sam_adc_s *priv = (struct sam_adc_s *)dev->ad_priv;
#endif

  ainfo("enable=%d\n", enable);

#ifdef CONFIG_SAMA5_ADC_DMA
  /* Ignore redundant requests */

  if (priv->enabled != enable)
    {
      /* Set a flag.  If disabling, the DMA sequence will terminate at the
       * completion of the next DMA.
       */

      priv->enabled = enable;

      /* If enabling, then we need to restart the DMA transfer */

      sam_adc_dmastart(priv);
    }

#else
  /* Are we enabling or disabling? */

  if (enable)
    {
      /* Enable channel interrupts */

      sam_adc_putreg(priv, SAM_ADC_IER, SAMA5_CHAN_ENABLE);
    }
  else
    {
      /* Disable channel interrupts */

      sam_adc_putreg(priv, SAM_ADC_IDR, ADC_INT_EOCALL);
    }
#endif
}

/****************************************************************************
 * Name: sam_adc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int sam_adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg)
{
#ifdef CONFIG_SAMA5_ADC_SWTRIG
  struct sam_adc_s *priv = (struct sam_adc_s *)dev->ad_priv;
#endif
  int ret = OK;

  ainfo("cmd=%d arg=%ld\n", cmd, arg);

  switch (cmd)
    {
#ifdef CONFIG_SAMA5_ADC_SWTRIG
      case ANIOC_TRIGGER: /* Software trigger */
        {
          sam_adc_putreg(priv, SAM_ADC_CR, ADC_CR_START); /* Start conversion */
        }
        break;
#endif

      /* Unsupported or invalid command */

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Initialization/Configuration
 ****************************************************************************/

/****************************************************************************
 * Name: sam_adc_settimer
 *
 * Description:
 *   Configure a timer to trigger the sampling periodically
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_ADC_TIOATRIG
static int sam_adc_settimer(struct sam_adc_s *priv, uint32_t frequency,
                            int channel)
{
  uint32_t div;
  uint32_t tcclks;
  uint32_t mode;
  uint32_t fdiv;
  uint32_t regval;
  int ret;

  ainfo("frequency=%ld channel=%d\n", (long)frequency, channel);
  DEBUGASSERT(priv && frequency > 0);

  /* Configure TC for a 1Hz frequency and trigger on RC compare. */

  ret = sam_tc_divisor(frequency, &div, &tcclks);
  if (ret < 0)
    {
      aerr("ERROR: sam_tc_divisor failed: %d\n", ret);
      return ret;
    }

  /* Set the timer/counter waveform mode the clock input selected by
   * sam_tc_divisor()
   */

  mode = ((tcclks << TC_CMR_TCCLKS_SHIFT) |  /* Use selected TCCLKS value */
          TC_CMR_WAVSEL_UPRC |               /* UP mode w/ trigger on RC Compare */
          TC_CMR_WAVE |                      /* Wave mode */
          TC_CMR_ACPA_CLEAR |                /* RA Compare Effect on TIOA: Clear */
          TC_CMR_ACPC_SET);                  /* RC effect on TIOA: Set */

  /* Now allocate and configure the channel */

  priv->tc = sam_tc_allocate(channel, mode);
  if (!priv->tc)
    {
      aerr("ERROR: Failed to allocate channel %d mode %08x\n",
            channel, mode);
      return -EINVAL;
    }

  /* The divider returned by sam_tc_divisor() is the reload value that will
   * achieve a 1Hz rate.  We need to multiply this to get the desired
   * frequency.  sam_tc_divisor() should have already assure that we can
   * do this without overflowing a 32-bit unsigned integer.
   */

  fdiv = div * frequency;
  DEBUGASSERT(div > 0 && div <= fdiv); /* Will check for integer overflow */

  /* Calculate the actual counter value from this divider and the tc input
   * frequency.
   */

  regval = sam_tc_infreq() / fdiv;

  /* Set up TC_RA and TC_RC.  The frequency is determined by RA and RC:
   * TIOA is cleared on RA match; TIOA is set on RC match.
   */

  sam_tc_setregister(priv->tc, TC_REGA, regval >> 1);
  sam_tc_setregister(priv->tc, TC_REGC, regval);

  /* And start the timer */

  sam_tc_start(priv->tc);
  return OK;
}
#endif

/****************************************************************************
 * Name: sam_adc_freetimer
 *
 * Description:
 *   Configure a timer to trigger the sampling periodically
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_ADC_TIOATRIG
static void sam_adc_freetimer(struct sam_adc_s *priv)
{
  /* Is a timer allocated? */

  ainfo("tc=%p\n", priv->tc);

  if (priv->tc)
    {
      /* Yes.. stop it and free it */

      sam_tc_stop(priv->tc);
      sam_tc_free(priv->tc);
      priv->tc = NULL;
    }
}
#endif

/****************************************************************************
 * Name: sam_adc_trigger
 *
 * Description:
 *   Configure trigger mode and start conversion.
 *
 ****************************************************************************/

static int sam_adc_trigger(struct sam_adc_s *priv)
{
  uint32_t regval;
  int ret = OK;

#if defined(CONFIG_SAMA5_ADC_SWTRIG)
  ainfo("Setup software trigger\n");

  /* Configure the software trigger */

  regval  = sam_adc_getreg(priv, SAM_ADC_MR);
  regval &= ~ADC_MR_TRGSEL_MASK;
  sam_adc_putreg(priv, SAM_ADC_MR, regval);

  /* No trigger, only software trigger can start conversions */

  regval  = sam_adc_getreg(priv, SAM_ADC_TRGR);
  regval &= ~ADC_TRGR_TRGMOD_MASK;
  regval |= ADC_TRGR_TRGMOD_NOTRIG;
  sam_adc_putreg(priv, SAM_ADC_TRGR, regval);

#elif defined(CONFIG_SAMA5_ADC_ADTRG)
  ainfo("Setup ADTRG trigger\n");

  /* Configure the trigger via the external ADTRG signal */

  regval  = sam_adc_getreg(priv, SAM_ADC_MR);
  regval &= ~ADC_MR_TRGSEL_MASK;
  regval |= ADC_MR_TRGSEL_ADC_ADTRIG;
  sam_adc_putreg(priv, SAM_ADC_MR, regval);

  /* External trigger edge selection */

  regval  = sam_adc_getreg(priv, SAM_ADC_TRGR);
  regval &= ~ADC_TRGR_TRGMOD_MASK;

#if defined(CONFIG_SAMA5_ADC_ADTRG_RISING)
  regval |= ADC_TRGR_TRGMOD_EXTRISE;
#elif defined(CONFIG_SAMA5_ADC_ADTRG_FALLING)
  regval |= ADC_TRGR_TRGMOD_EXTFALL;
#elif defined(CONFIG_SAMA5_ADC_ADTRG_BOTH)
  regval |= ADC_TRGR_TRGMOD_EXTBOTH;
#else
#  error External trigger edge not defined
#endif

  sam_adc_putreg(priv, SAM_ADC_TRGR, regval);

#elif defined(CONFIG_SAMA5_ADC_TIOATRIG)
  ainfo("Setup timer/counter trigger\n");

  /* Start the timer */

#if defined(CONFIG_SAMA5_ADC_TIOA0TRIG)
  ret = sam_adc_settimer(priv, CONFIG_SAMA5_ADC_TIOAFREQ, TC_CHAN0);
#elif defined(CONFIG_SAMA5_ADC_TIOA1TRIG)
  ret = sam_adc_settimer(priv, CONFIG_SAMA5_ADC_TIOAFREQ, TC_CHAN1);
#elif defined(CONFIG_SAMA5_ADC_TIOA2TRIG)
  ret = sam_adc_settimer(priv, CONFIG_SAMA5_ADC_TIOAFREQ, TC_CHAN2);
#else
#  error Timer/counter for trigger not defined
  ret = -ENOSYS;
#endif
  if (ret < 0)
    {
      aerr("ERROR: sam_adc_settimer failed: %d\n", ret);
      return ret;
    }

  /* Configure to trigger using Timer/counter 0, channel 1, 2, or 3.
   * NOTE: This trigger option depends on having properly configuer
   * timer/counter 0 to provide this output.  That is done independently
   * the timer/counter driver.
   */

  /* Set TIOAn trigger where n=0, 1, or 2 */

  regval  = sam_adc_getreg(priv, SAM_ADC_MR);
  regval &= ~ADC_MR_TRGSEL_MASK;

#if defined(CONFIG_SAMA5_ADC_TIOA0TRIG)
  regval |= ADC_MR_TRGSEL_TIOA0;   /* Timer/counter 0 channel 0 output A */
#elif defined(CONFIG_SAMA5_ADC_TIOA1TRIG)
  regval |= ADC_MR_TRGSEL_TIOA1;   /* Timer/counter 0 channel 1 output A */
#elif defined(CONFIG_SAMA5_ADC_TIOA2TRIG)
  regval |= ADC_MR_TRGSEL_TIOA2;   /* Timer/counter 0 channel 2 output A */
#else
#  error Timer/counter for trigger not defined
#endif

  sam_adc_putreg(priv, SAM_ADC_MR, regval);

  /* Timer trigger edge selection */

  regval  = sam_adc_getreg(priv, SAM_ADC_TRGR);
  regval &= ~ADC_TRGR_TRGMOD_MASK;

#if defined(CONFIG_SAMA5_ADC_TIOA_RISING)
  regval |= ADC_TRGR_TRGMOD_EXTRISE;
#elif defined(CONFIG_SAMA5_ADC_TIOA_FALLING)
  regval |= ADC_TRGR_TRGMOD_EXTFALL;
#elif defined(CONFIG_SAMA5_ADC_TIOA_BOTH)
  regval |= ADC_TRGR_TRGMOD_EXTBOTH;
#else
#  error External trigger edge not defined
#endif

  sam_adc_putreg(priv, SAM_ADC_TRGR, regval);

#else
#  error "Undefined ADC trigger"
#endif

  return ret;
}

/****************************************************************************
 * Name: sam_adc_autocalibrate
 *
 * Description:
 *   Perform ADC auto-calibration.
 *
 ****************************************************************************/

static void sam_adc_autocalibrate(struct sam_adc_s *priv)
{
#ifdef CONFIG_SAMA5_ADC_AUTOCALIB
  uint32_t regval;

  ainfo("Entry\n");

  /* Launch an automatic calibration of the ADC cell on next sequence */

  regval = sam_adc_getreg(priv, SAM_ADC_CR);
  regval |= ADC_CR_AUTOCAL;
  sam_adc_putreg(priv, SAM_ADC_CR, regval);

  /* Wait for auto calibration to complete */

  while ((sam_adc_getreg(priv, SAM_ADC_ISR) &
          ADC_ISR_EOCAL) != ADC_ISR_EOCAL);
#endif
}

/****************************************************************************
 * Name: sam_adc_offset
 *
 * Description:
 *   Configure ADC offset.  Also while we are modifying the COR register,
 *   configure differential mode if selected.
 *
 ****************************************************************************/

static void sam_adc_offset(struct sam_adc_s *priv)
{
  uint32_t regval = 0;

  ainfo("Entry\n");

#ifdef CONFIG_SAMA5_ADC_ANARCH
  /* Set the offset for each enabled channel.  This centers the analog signal
   * on Vrefin/2 before the gain scaling. The Offset applied is:
   * (G-1)Vrefin/2 where G is the gain applied. The default is no offset.
   */

#if defined(CONFIG_SAMA5_ADC_CHAN0) && defined(CONFIG_SAMA5_ADC_OFFSET0)
  regval |= ADC_COR_OFF0;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN1) && defined(CONFIG_SAMA5_ADC_OFFSET1)
  regval |= ADC_COR_OFF1;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN2) && defined(CONFIG_SAMA5_ADC_OFFSET2)
  regval |= ADC_COR_OFF2;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN3) && defined(CONFIG_SAMA5_ADC_OFFSET3)
  regval |= ADC_COR_OFF3;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN4) && defined(CONFIG_SAMA5_ADC_OFFSET4)
  regval |= ADC_COR_OFF4;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN5) && defined(CONFIG_SAMA5_ADC_OFFSET5)
  regval |= ADC_COR_OFF5;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN6) && defined(CONFIG_SAMA5_ADC_OFFSET6)
  regval |= ADC_COR_OFF6;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN7) && defined(CONFIG_SAMA5_ADC_OFFSET7)
  regval |= ADC_COR_OFF7;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN8) && defined(CONFIG_SAMA5_ADC_OFFSET8)
  regval |= ADC_COR_OFF8;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN9) && defined(CONFIG_SAMA5_ADC_OFFSET9)
  regval |= ADC_COR_OFF9;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN10) && defined(CONFIG_SAMA5_ADC_OFFSET10)
  regval |= ADC_COR_OFF10;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN11) && defined(CONFIG_SAMA5_ADC_OFFSET11)
  regval |= ADC_COR_OFF11;
#endif

  /* Set the differential mode of operation for each enabled channel.
   * The default is single-ended operation.
   */

#if defined(CONFIG_SAMA5_ADC_CHAN0) && defined(CONFIG_SAMA5_ADC_DIFFMODE0)
  regval |= ADC_COR_DIFF0;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN1) && defined(CONFIG_SAMA5_ADC_DIFFMODE1)
  regval |= ADC_COR_DIFF1;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN2) && defined(CONFIG_SAMA5_ADC_DIFFMODE2)
  regval |= ADC_COR_DIFF2;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN3) && defined(CONFIG_SAMA5_ADC_DIFFMODE3)
  regval |= ADC_COR_DIFF3;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN4) && defined(CONFIG_SAMA5_ADC_DIFFMODE4)
  regval |= ADC_COR_DIFF4;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN5) && defined(CONFIG_SAMA5_ADC_DIFFMODE5)
  regval |= ADC_COR_DIFF5;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN6) && defined(CONFIG_SAMA5_ADC_DIFFMODE6)
  regval |= ADC_COR_DIFF6;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN7) && defined(CONFIG_SAMA5_ADC_DIFFMODE7)
  regval |= ADC_COR_DIFF7;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN8) && defined(CONFIG_SAMA5_ADC_DIFFMODE8)
  regval |= ADC_COR_DIFF8;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN9) && defined(CONFIG_SAMA5_ADC_DIFFMODE9)
  regval |= ADC_COR_DIFF9;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN10) && defined(CONFIG_SAMA5_ADC_DIFFMODE10)
  regval |= ADC_COR_DIFF10;
#endif
#if defined(CONFIG_SAMA5_ADC_CHAN11) && defined(CONFIG_SAMA5_ADC_DIFFMODE11)
  regval |= ADC_COR_DIFF11;
#endif

#else
  /* Set offset and differential mode only on channel 0.  This will be
   * used for all channel.
   */

#ifdef CONFIG_SAMA5_ADC_OFFSET
  regval |= ADC_COR_OFF0;
#endif
#ifdef CONFIG_SAMA5_ADC_DIFFMODE
  regval |= ADC_COR_DIFF0;
#endif
#endif

  /* Save the updated COR register value */

  sam_adc_putreg(priv, SAM_ADC_COR, regval);
}

/****************************************************************************
 * Name: sam_adc_gain
 *
 * Description:
 *   Configure ADC gain.
 *
 ****************************************************************************/

static void sam_adc_gain(struct sam_adc_s *priv)
{
#ifdef CONFIG_SAMA5_ADC_ANARCH
  uint32_t regval;

  ainfo("Entry\n");

  /* Set the gain for each enabled channel */

  regval = 0;

#ifdef CONFIG_SAMA5_ADC_CHAN0
  regval |= ADC_CGR_GAIN0(CONFIG_SAMA5_ADC_GAIN0);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN1
  regval |= ADC_CGR_GAIN1(CONFIG_SAMA5_ADC_GAIN1);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN2
  regval |= ADC_CGR_GAIN2(CONFIG_SAMA5_ADC_GAIN2);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN3
  regval |= ADC_CGR_GAIN3(CONFIG_SAMA5_ADC_GAIN3);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN4
  regval |= ADC_CGR_GAIN4(CONFIG_SAMA5_ADC_GAIN4);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN5
  regval |= ADC_CGR_GAIN5(CONFIG_SAMA5_ADC_GAIN5);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN6
  regval |= ADC_CGR_GAIN6(CONFIG_SAMA5_ADC_GAIN6);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN7
  regval |= ADC_CGR_GAIN7(CONFIG_SAMA5_ADC_GAIN7);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN8
  regval |= ADC_CGR_GAIN8(CONFIG_SAMA5_ADC_GAIN8);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN9
  regval |= ADC_CGR_GAIN9(CONFIG_SAMA5_ADC_GAIN9);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN10
  regval |= ADC_CGR_GAIN10(CONFIG_SAMA5_ADC_GAIN10);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN11
  regval |= ADC_CGR_GAIN11(CONFIG_SAMA5_ADC_GAIN11);
#endif

  sam_adc_putreg(priv, SAM_ADC_CGR, regval);

#else
  ainfo("Gain=%d\n", CONFIG_SAMA5_ADC_GAIN);

  /* Set GAIN0 only.  GAIN0 will be used for all channels. */

  sam_adc_putreg(priv, SAM_ADC_CGR, ADC_CGR_GAIN0(CONFIG_SAMA5_ADC_GAIN));
#endif
}

/****************************************************************************
 * Name: sam_adc_analogchange
 *
 * Description:
 *   Configure analog change.  This features permits different analog
 *   settings per channel.
 *
 ****************************************************************************/

static void sam_adc_analogchange(struct sam_adc_s *priv)
{
  uint32_t regval;

  ainfo("Entry\n");

  /* Enable/disable the analog change feature */

  regval = sam_adc_getreg(priv, SAM_ADC_MR);

#ifdef CONFIG_SAMA5_ADC_ANARCH
  /* Disable analog change: No analog change on channel switching: DIFF0,
   * GAIN0 and OFF0 are used for all channels.
   */

  regval |= ADC_MR_ANACH;
#else
  /* Enable analog change: Allows different analog settings for each
   * channel using the ADC_CGR and ADC_COR Registers.
   */

  regval &= ~ADC_MR_ANACH;
#endif

  sam_adc_putreg(priv, SAM_ADC_MR, regval);
}

/****************************************************************************
 * Name: sam_adc_sequencer
 *
 * Description:
 *   Configure and enable the sequencer
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_ADC_SEQUENCER
static void sam_adc_setseqr(int chan, uint32_t *seqr1, uint32_t *seqr2,
                            int seq)
{
  if (seq > 8)
    {
      *seqr2 |= ADC_SEQR2_USCH(seq, chan);
    }
  else
    {
      *seqr1 |= ADC_SEQR1_USCH(seq, chan);
    }

  ainfo("chan=%d seqr1=%08x seqr2=%08x seq=%d\n", chan, *seqr1, *seqr2, seq);
}
#endif

static void sam_adc_sequencer(struct sam_adc_s *priv)
{
#ifdef CONFIG_SAMA5_ADC_SEQUENCER
  uint32_t regval;
  uint32_t seqr1;
  uint32_t seqr2;
  int seq;

  ainfo("Setup sequencer\n");

  /* Set user configured channel sequence */

  seqr1 = 0;
  seqr2 = 0;
  seq   = 1;

#ifdef CONFIG_SAMA5_ADC_CHAN0
  sam_adc_setseqr(0, &seqr1, &seqr2, seq++);
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN1
  sam_adc_setseqr(1, &seqr1, &seqr2, seq++);
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN2
  sam_adc_setseqr(2, &seqr1, &seqr2, seq++);
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN3
  sam_adc_setseqr(3, &seqr1, &seqr2, seq++);
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN4
  sam_adc_setseqr(4, &seqr1, &seqr2, seq++);
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN5
  sam_adc_setseqr(5, &seqr1, &seqr2, seq++);
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN6
  sam_adc_setseqr(6, &seqr1, &seqr2, seq++);
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN7
  sam_adc_setseqr(7, &seqr1, &seqr2, seq++);
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN8
  sam_adc_setseqr(8, &seqr1, &seqr2, seq++);
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN9
  sam_adc_setseqr(9, &seqr1, &seqr2, seq++);
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN10
  sam_adc_setseqr(10, &seqr1, &seqr2, seq++);
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN11
  sam_adc_setseqr(11, &seqr1, &seqr2, seq++);
#endif

  /* If not all channels are used, then program an unused channel number
   * into the remaining slots.  If we don't do this, we will get multiple
   * samples for the enabled channels.
   */

#ifdef SAMA5_ADC_UNUSED
  for (; seq < 9; seq++)
    {
      seqr1 |= ADC_SEQR1_USCH(seq, SAMA5_ADC_UNUSED);
    }

  for (; seq < 12; seq++)
    {
      seqr2 |= ADC_SEQR2_USCH(seq, SAMA5_ADC_UNUSED);
    }
#endif

  /* Save the new values to the SEQR1 and SEQR2 registers */

  sam_adc_putreg(priv, SAM_ADC_SEQR1, seqr1);
  sam_adc_putreg(priv, SAM_ADC_SEQR2, seqr2);

  /* Enable sequencer.  Any channel that is not enabled will be skipped by
   * the sequencer (that is why we programmed the unused channels above.
   */

  regval  = sam_adc_getreg(priv, SAM_ADC_MR);
  regval |= ADC_MR_USEQ;
  sam_adc_putreg(priv, SAM_ADC_MR, regval);

#else
  uint32_t regval;

  ainfo("Disable sequencer\n");

  /* Disable the sequencer */

  regval  = sam_adc_getreg(priv, SAM_ADC_MR);
  regval &= ~ADC_MR_USEQ;
  sam_adc_putreg(priv, SAM_ADC_MR, regval);

#endif
}

/****************************************************************************
 * Name: sam_adc_channels
 *
 * Description:
 *   Configure and enable the channels
 *
 ****************************************************************************/

static void sam_adc_channels(struct sam_adc_s *priv)
{
  uint32_t regval;

  ainfo("Entry\n");

  /* Enable channels. */

  regval = 0;

#ifdef CONFIG_SAMA5_ADC_CHAN0
  regval |= ADC_CH0;
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN1
  regval |= ADC_CH1;
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN2
  regval |= ADC_CH2;
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN3
  regval |= ADC_CH3;
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN4
  regval |= ADC_CH4;
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN5
  regval |= ADC_CH5;
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN6
  regval |= ADC_CH6;
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN7
  regval |= ADC_CH7;
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN8
  regval |= ADC_CH8;
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN9
  regval |= ADC_CH9;
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN10
  regval |= ADC_CH10;
#endif

#ifdef CONFIG_SAMA5_ADC_CHAN11
  regval |= ADC_CH11;
#endif

  sam_adc_putreg(priv, SAM_ADC_CHER, regval);
}
#endif /* SAMA5_ADC_HAVE_CHANNELS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_adc_initialize
 *
 * Description:
 *   Initialize the adc
 *
 * Returned Value:
 *   Valid can device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *sam_adc_initialize(void)
{
  struct sam_adc_s *priv = &g_adcpriv;
  uint32_t regval;
  uint32_t mck;
  int ret;

  /* Have we already been initialized?  If yes, than just hand out the
   * interface one more time.
   */

  if (!priv->initialized)
    {
      ainfo("Initializing...\n");

      /* Disable ADC peripheral clock */

      sam_adc_disableclk();

      /* Configure ADC pins */

#ifdef CONFIG_SAMA5_ADC_CHAN0
      sam_configpio(PIO_ADC_AD0);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN1
      sam_configpio(PIO_ADC_AD1);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN2
      sam_configpio(PIO_ADC_AD2);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN3
      sam_configpio(PIO_ADC_AD3);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN4
      sam_configpio(PIO_ADC_AD4);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN5
      sam_configpio(PIO_ADC_AD5);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN6
      sam_configpio(PIO_ADC_AD6);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN7
      sam_configpio(PIO_ADC_AD7);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN8
      sam_configpio(PIO_ADC_AD8);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN9
      sam_configpio(PIO_ADC_AD9);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN10
      sam_configpio(PIO_ADC_AD10);
#endif
#ifdef CONFIG_SAMA5_ADC_CHAN11
      sam_configpio(PIO_ADC_AD11);
#endif

#ifdef CONFIG_SAMA5_ADC_ADTRG
      sam_configpio(PIO_ADC_TRG);
#endif

      /* Initialize the public ADC device data structure */

#ifdef SAMA5_ADC_HAVE_CHANNELS
      g_adcdev.ad_ops  = &g_adcops;
      priv->dev = &g_adcdev;
#endif

      g_adcdev.ad_priv = priv;

      /* Initialize the private ADC device data structure */

      nxsem_init(&priv->exclsem,  0, 1);
      priv->cb  = NULL;

#ifdef CONFIG_SAMA5_ADC_DMA
      /* Allocate a DMA channel from DMAC1 */

      priv->dma = sam_dmachannel(1, DMA_FLAGS);
      DEBUGASSERT(priv->dma);
#endif

      /* Determine the maximum ADC peripheral clock frequency */

      mck = BOARD_MCK_FREQUENCY;
      if (mck <= SAM_ADC_MAXPERCLK)
        {
          priv->frequency = mck;
          regval          = PMC_PCR_DIV1;
        }
      else if ((mck >> 1) <= SAM_ADC_MAXPERCLK)
        {
          priv->frequency = (mck >> 1);
          regval          = PMC_PCR_DIV2;
        }
      else if ((mck >> 2) <= SAM_ADC_MAXPERCLK)
        {
          priv->frequency = (mck >> 2);
          regval          = PMC_PCR_DIV4;
        }
      else if ((mck >> 3) <= SAM_ADC_MAXPERCLK)
        {
          priv->frequency = (mck >> 3);
          regval          = PMC_PCR_DIV8;
        }
      else
        {
          aerr("ERROR: Cannot realize ADC input frequency\n");
          return NULL;
        }

      /* Set the maximum ADC peripheral clock frequency */

      regval |= PMC_PCR_PID(SAM_PID_ADC) | PMC_PCR_CMD | PMC_PCR_EN;
      sam_adc_putreg(priv, SAM_PMC_PCR, regval);

      /* Enable the ADC peripheral clock */

      sam_adc_enableclk();

      /* Reset the ADC controller */

      sam_adc_putreg(priv, SAM_ADC_CR, ADC_CR_SWRST);

      /* Reset Mode Register */

      sam_adc_putreg(priv, SAM_ADC_MR, 0);

      /* Set the MCK clock prescaler: ADCClock = MCK / ((PRESCAL+1)*2) */

      regval  = sam_adc_getreg(priv, SAM_ADC_MR);
      regval &= ~ADC_MR_PRESCAL_MASK;
      regval |=  ADC_MR_PRESCAL(BOARD_ADC_PRESCAL);
      sam_adc_putreg(priv, SAM_ADC_MR, regval);

      /* Formula:
       *     Startup  Time = startup value / ADCClock
       *     Transfer Time = (TRANSFER * 2 + 3) / ADCClock
       *     Tracking Time = (TRACKTIM + 1) / ADCClock
       *     Settling Time = settling value / ADCClock
       * For example, ADC clock = 6MHz (166.7 ns)
       *     Startup time = 512 / 6MHz = 85.3 us
       *     Transfer Time = (1 * 2 + 3) / 6MHz = 833.3 ns
       *     Tracking Time = (0 + 1) / 6MHz = 166.7 ns
       *     Settling Time = 3 / 6MHz = 500 ns
       */

      /* Set ADC timing */

      regval  = sam_adc_getreg(priv, SAM_ADC_MR);
      regval &= ~(ADC_MR_STARTUP_MASK | ADC_MR_TRACKTIM_MASK |
                ADC_MR_SETTLING_MASK);
      regval |= (ADC_MR_STARTUP_512 | ADC_MR_TRACKTIM(0) |
                ADC_MR_SETTLING_17);
      sam_adc_putreg(priv, SAM_ADC_MR, regval);

      /* Attach the ADC interrupt */

      ret = irq_attach(SAM_IRQ_ADC, sam_adc_interrupt, NULL);
      if (ret < 0)
        {
          aerr("ERROR: Failed to attach IRQ %d: %d\n", SAM_IRQ_ADC, ret);
          return NULL;
        }

      /* Disable all ADC interrupts at the source */

      sam_adc_putreg(priv, SAM_ADC_IDR, ADC_INT_ALL);

      /* Enable the ADC interrupt at the AIC */

      up_enable_irq(SAM_IRQ_ADC);

      /* Now we are initialized */

      priv->initialized = true;
    }

  /* Return a pointer to the device structure */

  ainfo("Returning %p\n", &g_adcdev);
  return &g_adcdev;
}

/****************************************************************************
 * Name: sam_adc_lock
 *
 * Description:
 *   Get exclusive access to the ADC interface
 *
 ****************************************************************************/

int sam_adc_lock(FAR struct sam_adc_s *priv)
{
  ainfo("Locking\n");
  return nxsem_wait_uninterruptible(&priv->exclsem);
}

/****************************************************************************
 * Name: sam_adc_unlock
 *
 * Description:
 *   Relinquish the lock on the ADC interface
 *
 ****************************************************************************/

void sam_adc_unlock(FAR struct sam_adc_s *priv)
{
  ainfo("Unlocking\n");
  nxsem_post(&priv->exclsem);
}

/****************************************************************************
 * Name: sam_adc_getreg
 *
 * Description:
 *  Read any 32-bit register using an absolute address.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_ADC_REGDEBUG
uint32_t sam_adc_getreg(struct sam_adc_s *priv, uintptr_t address)
{
  uint32_t regval = getreg32(address);

  if (sam_adc_checkreg(priv, false, regval, address))
    {
      ainfo("%08x->%08x\n", address, regval);
    }

  return regval;
}
#endif

/****************************************************************************
 * Name: sam_adc_putreg
 *
 * Description:
 *  Write to any 32-bit register using an absolute address.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_ADC_REGDEBUG
void sam_adc_putreg(struct sam_adc_s *priv, uintptr_t address,
                    uint32_t regval)
{
  if (sam_adc_checkreg(priv, true, regval, address))
    {
      ainfo("%08x<-%08x\n", address, regval);
    }

  putreg32(regval, address);
}
#endif

#endif /* CONFIG_SAMA5_ADC */

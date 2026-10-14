/****************************************************************************
 * arch/arm/src/rm57/rm57_mibspi.c
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

/* See rm57_mibspi.h for what this driver covers.  The register map is in
 * hardware/rm57_mibspi.h; the initialization order follows SPNU562A
 * section 27.2.11.1 and HALCoGen's mibspiInit().
 *
 * SPIFMTn.PHASE is the inverse of CPHA (Table 27-3), so the four SPI modes
 * do not map onto POLARITY/PHASE the obvious way round - see
 * mibspi_setformat().
 *
 * Only data format 0 is used.  A module has four, selectable per word, but
 * the upper half sets mode, width and frequency again whenever it takes
 * the bus, so one cached format is enough and keeps the state in a single
 * register.
 *
 * In multi-buffer mode the module runs as a 3-pin master, which restricts
 * it to transfer group 0 (SPNU562A section 27.2.6.7).  TG0 always starts at
 * buffer 0 and is retriggered by software once per pass; where it ends is
 * set through the start pointer of TG1, a dummy group that is never
 * enabled.  Writing that pointer as the pass length, 128 included, is also
 * the workaround for errata MIBSPI#136, which only bites when a full
 * 128-buffer group is expressed as PSTART = 0.  On MibSPI1, in extended
 * mode, a pass of 256 cannot be expressed at all, so its passes stop at
 * 255.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/param.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/clock.h>
#include <nuttx/init.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/rm57_mibspi.h"
#include "hardware/rm57l843_memorymap.h"
#include "rm57_mibspi.h"

#ifdef CONFIG_RM57_MIBSPI_DMA
#  include "hardware/rm57_dma.h"
#  include "rm57_dma.h"
#endif

#ifdef CONFIG_RM57_MIBSPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Power-up format, until the upper half asks for something else */

#define RM57_MIBSPI_DEFFREQ     400000
#define RM57_MIBSPI_DEFBITS     8

/* SPICLK = VCLK / (PRESCALE + 1), except that PRESCALE = 0 also divides by
 * two.  The divider range is therefore 2 to 2048.
 */

#define RM57_MIBSPI_MINDIV      2
#define RM57_MIBSPI_MAXDIV      (MIBSPI_EXTPRESCALE_MAX + 1)
#define RM57_MIBSPI_MAXDIV8     (MIBSPI_PRESCALE_MAX + 1)

/* Control field loaded with every word.  CSHOLD keeps the module from
 * inserting its chip select setup/hold gaps between consecutive words;
 * with no SPICS pin in functional mode it has no other effect.
 */

#define RM57_MIBSPI_DAT1_CTRL   (MIBSPI_DAT1_CSHOLD | MIBSPI_DAT1_DFSEL(0))

/* The upper half of SPIDAT1, for control-field-only writes.  This is a
 * little-endian device, so it is the second 16-bit word of the register.
 */

#define RM57_MIBSPI_DAT1_CTRL_OFFSET  (RM57_MIBSPI_DAT1_OFFSET + 2)

/* Pins that must be functional for the state machine to leave reset
 * (SPNU562A section 27.2.7).
 */

#define RM57_MIBSPI_FUNCPINS    (MIBSPI_PC_CLK | MIBSPI_PC_SIMO0 | \
                                 MIBSPI_PC_SOMI0)

/* Polling bound: iterations without the transfer making any progress.  A
 * count rather than a time, so that exchanges also work with interrupts
 * disabled; at the slowest clock a 16-bit word takes about 440us, which
 * this exceeds by a couple of orders of magnitude.
 */

#define RM57_MIBSPI_POLL_LIMIT  1000000

/* At most this many words are in flight at once: one in the shift
 * register and one in the transmit buffer, which is also exactly what the
 * receive side can hold (SPIBUF plus the internal RXBUF) before it
 * overruns.
 */

#define RM57_MIBSPI_PIPELINE    2

/* Control field of every transmit buffer in multi-buffer mode.  CSHOLD is
 * added on all but the last buffer of a pass, as in compatibility mode.
 */

#define RM57_MIBSPI_TXRAM_CTRL  (MIBSPI_TXRAM_BUFMODE_CONTINUOUS | \
                                 MIBSPI_DAT1_DFSEL(0))

/* TG0 as a software-triggered one-shot group starting at buffer 0; setting
 * TGENA on top of this starts a pass (SPNU562A Table 27-43, TRIGEVT 7h).
 */

#define RM57_MIBSPI_TG0CTRL     (MIBSPI_TGCTRL_ONESHOT | \
                                 MIBSPI_TGCTRL_TRIGEVT_ALWAYS | \
                                 MIBSPI_TGCTRL_TRIGSRC_NONE | \
                                 MIBSPI_TGCTRL_PSTART(0))

/* A pass expected to finish sooner than this is polled for: blocking and
 * being woken would take about as long as the pass itself.
 */

#define RM57_MIBSPI_IRQ_MINUS   100

/* Compatibility-mode DMA.  The bounce buffers hold 16-bit words whatever
 * the word length, since the channels move halfwords to and from the data
 * registers; their size is rounded up to whole Cortex-R5 cache lines.
 */

#ifdef CONFIG_RM57_MIBSPI_DMA
#  define RM57_MIBSPI_CACHELINE 32
#  define RM57_MIBSPI_DMAWORDS  ((CONFIG_RM57_MIBSPI_DMABUFSIZE + 15) & ~15)
#endif

#ifdef CONFIG_RM57_MIBSPI1_MULTIBUF
#  define MIBSPI1_MULTIBUF      true
#else
#  define MIBSPI1_MULTIBUF      false
#endif

#ifdef CONFIG_RM57_MIBSPI2_MULTIBUF
#  define MIBSPI2_MULTIBUF      true
#else
#  define MIBSPI2_MULTIBUF      false
#endif

#ifdef CONFIG_RM57_MIBSPI3_MULTIBUF
#  define MIBSPI3_MULTIBUF      true
#else
#  define MIBSPI3_MULTIBUF      false
#endif

#ifdef CONFIG_RM57_MIBSPI4_MULTIBUF
#  define MIBSPI4_MULTIBUF      true
#else
#  define MIBSPI4_MULTIBUF      false
#endif

#ifdef CONFIG_RM57_MIBSPI5_MULTIBUF
#  define MIBSPI5_MULTIBUF      true
#else
#  define MIBSPI5_MULTIBUF      false
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rm57_mibspi_s
{
  struct spi_dev_s spidev;      /* Externally visible part; must be first */
  uint32_t         base;        /* Module register base */
  uint32_t         rambase;     /* Multi-buffer RAM base */
  uint16_t         nbuf;        /* Buffers in the RAM, in multi-buffer mode */
  uint8_t          irq;         /* Level 0 (high) interrupt line */
  uint8_t          bus;         /* Module number, 1-5 */
  uint8_t          ncs;         /* SPICS pins the module has */
  bool             multibuf;    /* Multi-buffer rather than compatibility */
  bool             initialized; /* Hardware has been set up */
  bool             lsbfirst;    /* Shift direction */
  uint8_t          mode;        /* enum spi_mode_e in effect */
  uint8_t          nbits;       /* Word length in effect */
  uint32_t         frequency;   /* Requested frequency */
  uint32_t         actual;      /* Frequency in effect */
  uint16_t         prescale;    /* Divider - 1 in effect */
  mutex_t          lock;        /* Held between SPI_LOCK(true) and (false) */
  sem_t            xfrdone;     /* Posted when a TG0 pass or DMA completes */
#ifdef CONFIG_RM57_MIBSPI_DMA
  uint8_t          txreq;       /* DMA request line, transmit */
  uint8_t          rxreq;       /* DMA request line, receive */
  uint16_t        *txbuf;       /* Bounce buffers; NULL without DMA */
  uint16_t        *rxbuf;
  DMA_HANDLE       txdma;       /* Channels; NULL when not allocated */
  DMA_HANDLE       rxdma;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t mibspi_getreg(struct rm57_mibspi_s *priv,
                                     unsigned int offset);
static inline void mibspi_putreg(struct rm57_mibspi_s *priv,
                                 unsigned int offset, uint32_t value);

static void mibspi_latchformat(struct rm57_mibspi_s *priv);
static void mibspi_setformat(struct rm57_mibspi_s *priv);
static void mibspi_drain(struct rm57_mibspi_s *priv);
static void mibspi_restart(struct rm57_mibspi_s *priv);
#ifdef CONFIG_RM57_MIBSPI_MULTIBUF
static int  mibspi_interrupt(int irq, void *context, void *arg);
static void mibspi_mbinitialize(struct rm57_mibspi_s *priv);
static int  mibspi_mbpass(struct rm57_mibspi_s *priv, const void *txbuffer,
                          void *rxbuffer, size_t offset, size_t nwords,
                          uint32_t *errors);
static void mibspi_mbexchange(struct rm57_mibspi_s *priv,
                              const void *txbuffer, void *rxbuffer,
                              size_t nwords);
#endif
#ifdef CONFIG_RM57_MIBSPI_DMA
static void mibspi_dmacallback(DMA_HANDLE handle, uint8_t status,
                               void *arg);
static void mibspi_dmainitialize(struct rm57_mibspi_s *priv);
static int  mibspi_dmablock(struct rm57_mibspi_s *priv,
                            const void *txbuffer, void *rxbuffer,
                            size_t offset, size_t nwords);
static void mibspi_dmaexchange(struct rm57_mibspi_s *priv,
                               const void *txbuffer, void *rxbuffer,
                               size_t nwords);
#endif
static void mibspi_cmexchange(struct rm57_mibspi_s *priv,
                              const void *txbuffer, void *rxbuffer,
                              size_t nwords);
static void mibspi_hwinitialize(struct rm57_mibspi_s *priv);
static struct rm57_mibspi_s *mibspi_instance(int bus);

static int  mibspi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t mibspi_setfrequency(struct spi_dev_s *dev,
                                    uint32_t frequency);
static void mibspi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void mibspi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int  mibspi_hwfeatures(struct spi_dev_s *dev,
                              spi_hwfeatures_t features);
#endif
static uint32_t mibspi_send(struct spi_dev_s *dev, uint32_t wd);
static void mibspi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                            void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void mibspi_sndblock(struct spi_dev_s *dev, const void *buffer,
                            size_t nwords);
static void mibspi_recvblock(struct spi_dev_s *dev, void *buffer,
                             size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* One spi_ops_s per bus, because select/status/cmddata belong to the board
 * and are named after the bus.
 */

#ifdef CONFIG_SPI_HWFEATURES
#  define MIBSPI_OPS_HWFEATURES   .hwfeatures = mibspi_hwfeatures,
#else
#  define MIBSPI_OPS_HWFEATURES
#endif

#ifdef CONFIG_SPI_CMDDATA
#  define MIBSPI_OPS_CMDDATA(n)   .cmddata = rm57_mibspi##n##cmddata,
#else
#  define MIBSPI_OPS_CMDDATA(n)
#endif

#ifdef CONFIG_SPI_EXCHANGE
#  define MIBSPI_OPS_TRANSFER     .exchange = mibspi_exchange,
#else
#  define MIBSPI_OPS_TRANSFER     .sndblock = mibspi_sndblock, \
                                  .recvblock = mibspi_recvblock,
#endif

#define MIBSPI_DEVICE(n, nchipsel, irqno, ramdepth) \
  static const struct spi_ops_s g_mibspi##n##ops = \
  { \
    .lock         = mibspi_lock, \
    .select       = rm57_mibspi##n##select, \
    .setfrequency = mibspi_setfrequency, \
    .setmode      = mibspi_setmode, \
    .setbits      = mibspi_setbits, \
    MIBSPI_OPS_HWFEATURES \
    .status       = rm57_mibspi##n##status, \
    MIBSPI_OPS_CMDDATA(n) \
    .send         = mibspi_send, \
    MIBSPI_OPS_TRANSFER \
  }; \
  static struct rm57_mibspi_s g_mibspi##n##dev = \
  { \
    .spidev.ops   = &g_mibspi##n##ops, \
    .base         = RM57_MIBSPI##n##_BASE, \
    .rambase      = RM57_MIBSPI##n##_RAM_BASE, \
    .nbuf         = ramdepth, \
    .irq          = irqno, \
    .bus          = n, \
    .ncs          = nchipsel, \
    .multibuf     = MIBSPI##n##_MULTIBUF, \
    .lock         = NXMUTEX_INITIALIZER, \
    .xfrdone      = NXSEM_INITIALIZER(0, SEM_PRIO_NONE), \
    MIBSPI##n##_DMAFIELDS \
  }

/* DMA bounce buffers, whole cache lines each so that cleaning and
 * invalidating them never touches anything else.
 */

#define MIBSPI_DMABUFFERS(n) \
  static uint16_t g_mibspi##n##txbuf[RM57_MIBSPI_DMAWORDS] \
    aligned_data(RM57_MIBSPI_CACHELINE); \
  static uint16_t g_mibspi##n##rxbuf[RM57_MIBSPI_DMAWORDS] \
    aligned_data(RM57_MIBSPI_CACHELINE)

#define MIBSPI_DMAFIELDS(n, tx, rx) \
    .txreq        = tx, \
    .rxreq        = rx, \
    .txbuf        = g_mibspi##n##txbuf, \
    .rxbuf        = g_mibspi##n##rxbuf,

#ifdef CONFIG_RM57_MIBSPI1_DMA
MIBSPI_DMABUFFERS(1);
#  define MIBSPI1_DMAFIELDS \
    MIBSPI_DMAFIELDS(1, DMA_REQ_MIBSPI1_0, DMA_REQ_MIBSPI1_1)
#else
#  define MIBSPI1_DMAFIELDS
#endif

#ifdef CONFIG_RM57_MIBSPI2_DMA
MIBSPI_DMABUFFERS(2);
#  define MIBSPI2_DMAFIELDS \
    MIBSPI_DMAFIELDS(2, DMA_REQ_MIBSPI2_0, DMA_REQ_MIBSPI2_1)
#else
#  define MIBSPI2_DMAFIELDS
#endif

#ifdef CONFIG_RM57_MIBSPI3_DMA
MIBSPI_DMABUFFERS(3);
#  define MIBSPI3_DMAFIELDS \
    MIBSPI_DMAFIELDS(3, DMA_REQ_MIBSPI3_0, DMA_REQ_MIBSPI3_1)
#else
#  define MIBSPI3_DMAFIELDS
#endif

/* MibSPI4 and MibSPI5 only have alternate DMA request lines */

#define MIBSPI4_DMAFIELDS
#define MIBSPI5_DMAFIELDS

#ifdef CONFIG_RM57_MIBSPI1
MIBSPI_DEVICE(1, 6, RM57_REQ_MIBSPI1HIGH, RM57_MIBSPI_NBUF_EXTENDED);
#endif

#ifdef CONFIG_RM57_MIBSPI2
MIBSPI_DEVICE(2, 2, RM57_REQ_SPI2HIGH, RM57_MIBSPI_NBUF);
#endif

#ifdef CONFIG_RM57_MIBSPI3
MIBSPI_DEVICE(3, 6, RM57_REQ_MIBSPI3HIGH, RM57_MIBSPI_NBUF);
#endif

#ifdef CONFIG_RM57_MIBSPI4
MIBSPI_DEVICE(4, 6, RM57_REQ_MIBSPI4HIGH, RM57_MIBSPI_NBUF);
#endif

#ifdef CONFIG_RM57_MIBSPI5
MIBSPI_DEVICE(5, 6, RM57_REQ_MIBSPI5HIGH, RM57_MIBSPI_NBUF);
#endif

/* Serializes first-time hardware initialization, which the bus and the
 * chip select helpers can both trigger.
 */

static mutex_t g_mibspi_initlock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mibspi_getreg and mibspi_putreg
 ****************************************************************************/

static inline uint32_t mibspi_getreg(struct rm57_mibspi_s *priv,
                                     unsigned int offset)
{
  return getreg32(priv->base + offset);
}

static inline void mibspi_putreg(struct rm57_mibspi_s *priv,
                                 unsigned int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: mibspi_latchformat
 *
 * Description:
 *   Make the module adopt data format 0 now.  A control-field-only write
 *   starts no transfer but selects the format, so SPICLK takes up its idle
 *   level here instead of at the first word.  SPNU562A documents this for
 *   SPIDAT1 (section 27.3.16) and for the transmit RAM (section 27.4.3).
 *
 ****************************************************************************/

static void mibspi_latchformat(struct rm57_mibspi_s *priv)
{
#ifdef CONFIG_RM57_MIBSPI_MULTIBUF
  if (priv->multibuf)
    {
      putreg16(RM57_MIBSPI_TXRAM_CTRL >> 16,
               priv->rambase + RM57_MIBSPI_TXRAM_OFFSET(0) + 2);
      return;
    }
#endif

  putreg16(RM57_MIBSPI_DAT1_CTRL >> 16,
           priv->base + RM57_MIBSPI_DAT1_CTRL_OFFSET);
}

/****************************************************************************
 * Name: mibspi_setformat
 *
 * Description:
 *   Program data format 0 from the cached frequency, mode, word length and
 *   shift direction, and record the frequency that results.
 *
 ****************************************************************************/

static void mibspi_setformat(struct rm57_mibspi_s *priv)
{
  uint32_t frequency = priv->frequency;
  uint32_t div;
  uint32_t fmt;
  uint32_t regval;

  /* Round the divider up so the clock never exceeds what was asked for */

  if (frequency == 0)
    {
      frequency = 1;
    }

  div = (BOARD_VCLK_FREQUENCY + frequency - 1) / frequency;
  if (div < RM57_MIBSPI_MINDIV)
    {
      div = RM57_MIBSPI_MINDIV;
    }
  else if (div > RM57_MIBSPI_MAXDIV)
    {
      div = RM57_MIBSPI_MAXDIV;
    }

  fmt = MIBSPI_FMT_CHARLEN(priv->nbits) | MIBSPI_FMT_DISCSTIMERS;

  /* CPOL is POLARITY; CPHA is the inverse of PHASE */

  switch (priv->mode)
    {
      default:
      case SPIDEV_MODE0: /* CPOL=0 CPHA=0 */
        fmt |= MIBSPI_FMT_PHASE;
        break;

      case SPIDEV_MODE1: /* CPOL=0 CPHA=1 */
        break;

      case SPIDEV_MODE2: /* CPOL=1 CPHA=0 */
        fmt |= MIBSPI_FMT_POLARITY | MIBSPI_FMT_PHASE;
        break;

      case SPIDEV_MODE3: /* CPOL=1 CPHA=1 */
        fmt |= MIBSPI_FMT_POLARITY;
        break;
    }

  if (priv->lsbfirst)
    {
      fmt |= MIBSPI_FMT_SHIFTDIR;
    }

  /* The low 8 bits of the divider go in SPIFMT0.  Dividers past 256 need
   * the extended prescale register, which has to be written second since
   * the SPIFMT0 write clears its upper bits.
   *
   * That register sits past the point where SPNU562A section 27.3.25 says
   * registers stop being accessible in compatibility mode, so the write is
   * verified rather than trusted: if it does not read back, fall back to
   * the slowest clock SPIFMT0 can produce on its own.
   */

  mibspi_putreg(priv, RM57_MIBSPI_FMT0_OFFSET,
                fmt | MIBSPI_FMT_PRESCALE((div - 1) & MIBSPI_PRESCALE_MAX));

  if (div > RM57_MIBSPI_MAXDIV8)
    {
      regval  = mibspi_getreg(priv, RM57_MIBSPI_EXTPRESCALE1_OFFSET);
      regval &= ~MIBSPI_EXTPRESCALE_LO_MASK;
      regval |= MIBSPI_EXTPRESCALE_LO(div - 1);
      mibspi_putreg(priv, RM57_MIBSPI_EXTPRESCALE1_OFFSET, regval);

      regval  = mibspi_getreg(priv, RM57_MIBSPI_EXTPRESCALE1_OFFSET);
      regval &= MIBSPI_EXTPRESCALE_LO_MASK;
      if (regval != MIBSPI_EXTPRESCALE_LO(div - 1))
        {
          spiwarn("MibSPI%d: extended prescale reads %03" PRIx32
                  ", wanted %03" PRIx32 "\n",
                  priv->bus, regval, div - 1);

          div = RM57_MIBSPI_MAXDIV8;
          mibspi_putreg(priv, RM57_MIBSPI_FMT0_OFFSET,
                        fmt | MIBSPI_FMT_PRESCALE(div - 1));
        }
    }

  mibspi_latchformat(priv);

  priv->prescale = div - 1;
  priv->actual   = BOARD_VCLK_FREQUENCY / div;

  spiinfo("MibSPI%d: FMT0=%08" PRIx32 " div=%" PRIu32 " %" PRIu32 " Hz\n",
          priv->bus, mibspi_getreg(priv, RM57_MIBSPI_FMT0_OFFSET), div,
          priv->actual);
}

/****************************************************************************
 * Name: mibspi_drain
 *
 * Description:
 *   Discard anything left in the receive path and clear the latched error
 *   flags.  Clearing a flag in SPIFLG does not clear its copy in SPIBUF;
 *   that only goes once SPIBUF has been read empty (SPNU562A Table 27-13).
 *
 ****************************************************************************/

static void mibspi_drain(struct rm57_mibspi_s *priv)
{
  int i;

  for (i = 0; i <= RM57_MIBSPI_PIPELINE; i++)
    {
      if ((mibspi_getreg(priv, RM57_MIBSPI_BUF_OFFSET) &
           MIBSPI_BUF_RXEMPTY) != 0)
        {
          break;
        }
    }

  mibspi_putreg(priv, RM57_MIBSPI_FLG_OFFSET, MIBSPI_INT_ALLERRORS);
}

/****************************************************************************
 * Name: mibspi_restart
 *
 * Description:
 *   Recover from a transfer that stopped making progress.  Dropping SPIEN
 *   forces both shift registers, the buffers behind them and SPIFLG back
 *   to their reset state without disturbing the configuration.  In
 *   multi-buffer mode TG0 is disabled first, so the sequencer does not
 *   pick the pass up again.  DMA requests go off before SPIEN does, the
 *   order errata MIBSPI#137 asks for.
 *
 ****************************************************************************/

static void mibspi_restart(struct rm57_mibspi_s *priv)
{
  uint32_t regval;

  mibspi_putreg(priv, RM57_MIBSPI_INT0_OFFSET, 0);

#ifdef CONFIG_RM57_MIBSPI_MULTIBUF
  if (priv->multibuf)
    {
      mibspi_putreg(priv, RM57_MIBSPI_TGCTRL_OFFSET(0), RM57_MIBSPI_TG0CTRL);
    }
#endif

  regval = mibspi_getreg(priv, RM57_MIBSPI_GCR1_OFFSET);
  mibspi_putreg(priv, RM57_MIBSPI_GCR1_OFFSET, regval & ~MIBSPI_GCR1_SPIEN);
  mibspi_putreg(priv, RM57_MIBSPI_GCR1_OFFSET, regval | MIBSPI_GCR1_SPIEN);

#ifdef CONFIG_RM57_MIBSPI_MULTIBUF
  if (priv->multibuf)
    {
      mibspi_putreg(priv, RM57_MIBSPI_TGINTFLG_OFFSET, 0xffffffff);
    }
#endif

  mibspi_latchformat(priv);
}

#ifdef CONFIG_RM57_MIBSPI_MULTIBUF
/****************************************************************************
 * Name: mibspi_interrupt
 *
 * Description:
 *   Level 0 interrupt.  The only source ever enabled is TG0 completion,
 *   and only while a pass is being waited for; it is turned off again
 *   here, so a late or spurious completion cannot leave a stale count on
 *   the semaphore.
 *
 ****************************************************************************/

static int mibspi_interrupt(int irq, void *context, void *arg)
{
  struct rm57_mibspi_s *priv = arg;

  if ((mibspi_getreg(priv, RM57_MIBSPI_TGINTFLG_OFFSET) &
       MIBSPI_TGINT_RDY(0)) != 0)
    {
      mibspi_putreg(priv, RM57_MIBSPI_TGITENCR_OFFSET, MIBSPI_TGINT_RDY(0));
      mibspi_putreg(priv, RM57_MIBSPI_TGINTFLG_OFFSET, MIBSPI_TGINT_RDY(0));
      nxsem_post(&priv->xfrdone);
    }

  return OK;
}

/****************************************************************************
 * Name: mibspi_mbinitialize
 *
 * Description:
 *   Transfer group set-up for multi-buffer mode.  The RAM has finished its
 *   automatic initialization by now, which would otherwise have reset
 *   these registers again (SPNU562A section 27.4.1).
 *
 ****************************************************************************/

static void mibspi_mbinitialize(struct rm57_mibspi_s *priv)
{
  int ret;

  /* Every TG interrupt off, on level 0, and nothing pending */

  mibspi_putreg(priv, RM57_MIBSPI_TGITENCR_OFFSET, 0xffffffff);
  mibspi_putreg(priv, RM57_MIBSPI_TGITLVCR_OFFSET, 0xffffffff);
  mibspi_putreg(priv, RM57_MIBSPI_TGINTFLG_OFFSET, 0xffffffff);

  /* The dummy TG1 runs to the end of the RAM; each pass moves its start,
   * and with it the end of TG0.
   */

  mibspi_putreg(priv, RM57_MIBSPI_LTGPEND_OFFSET,
                MIBSPI_LTGPEND_LPEND(priv->nbuf - 1));
  mibspi_putreg(priv, RM57_MIBSPI_TGCTRL_OFFSET(0), RM57_MIBSPI_TG0CTRL);

  ret = irq_attach(priv->irq, mibspi_interrupt, priv);
  if (ret < 0)
    {
      spierr("ERROR: MibSPI%d: irq_attach(%d) failed: %d\n",
             priv->bus, priv->irq, ret);
      return;
    }

  up_enable_irq(priv->irq);
}

/****************************************************************************
 * Name: mibspi_mbpass
 *
 * Description:
 *   Exchange up to one RAM's worth of words as a single TG0 pass.
 *
 * Input Parameters:
 *   priv     - Module
 *   txbuffer - Whole transmit buffer, or NULL
 *   rxbuffer - Whole receive buffer, or NULL
 *   offset   - Index of the first word of this pass in either buffer
 *   nwords   - Words in this pass: at least 1, at most the RAM depth and
 *              at most 255
 *   errors   - Accumulates the receive status flags of every word
 *
 * Returned Value:
 *   Zero on success; -ETIMEDOUT if the pass did not complete, in which
 *   case the module has been restarted.
 *
 ****************************************************************************/

static int mibspi_mbpass(struct rm57_mibspi_s *priv, const void *txbuffer,
                         void *rxbuffer, size_t offset, size_t nwords,
                         uint32_t *errors)
{
  const uint8_t *tx8 = (const uint8_t *)txbuffer + offset;
  const uint16_t *tx16 = (const uint16_t *)txbuffer + offset;
  uint8_t *rx8 = (uint8_t *)rxbuffer + offset;
  uint16_t *rx16 = (uint16_t *)rxbuffer + offset;
  bool extended = priv->nbuf > RM57_MIBSPI_NBUF;
  bool wide = priv->nbits > 8;
  uint32_t passus;
  uint32_t ctrl;
  uint32_t wd;
  uint32_t idle;
  bool useirq;
  size_t i;
  int tg;
  int ret;

  /* Transmit side: one 32-bit write per buffer sets control and data */

  for (i = 0; i < nwords; i++)
    {
      if (txbuffer == NULL)
        {
          wd = 0xffff;
        }
      else if (wide)
        {
          wd = tx16[i];
        }
      else
        {
          wd = tx8[i];
        }

      ctrl = RM57_MIBSPI_TXRAM_CTRL;
      if (i + 1 < nwords)
        {
          ctrl |= MIBSPI_DAT1_CSHOLD;
        }

      putreg32(ctrl | (wd & MIBSPI_DAT1_TXDATA_MASK),
               priv->rambase + RM57_MIBSPI_TXRAM_OFFSET(i));
    }

  /* TG0 ends where TG1 starts.  HALCoGen moves every later group's start
   * along with TG1's when it builds a single-group layout; do the same.
   */

  for (tg = 1; tg < RM57_NMIBSPI_TG; tg++)
    {
      mibspi_putreg(priv, RM57_MIBSPI_TGCTRL_OFFSET(tg),
                    MIBSPI_TGCTRL_PSTART(nwords));
    }

  mibspi_putreg(priv, RM57_MIBSPI_TGINTFLG_OFFSET, MIBSPI_TGINT_RDY(0));

  /* Wait in the interrupt only where it pays and where blocking is
   * possible at all.
   */

  passus = (uint32_t)(((uint64_t)nwords * priv->nbits * USEC_PER_SEC +
                       priv->actual - 1) / priv->actual);
  useirq = passus >= RM57_MIBSPI_IRQ_MINUS && OSINIT_OS_READY() &&
           !up_interrupt_context();

  if (useirq)
    {
      mibspi_putreg(priv, RM57_MIBSPI_TGITENST_OFFSET, MIBSPI_TGINT_RDY(0));
    }

  mibspi_putreg(priv, RM57_MIBSPI_TGCTRL_OFFSET(0),
                RM57_MIBSPI_TG0CTRL | MIBSPI_TGCTRL_TGENA);

  if (useirq)
    {
      /* Four times the nominal pass time, plus two ticks of slack for the
       * tick boundary and the wake-up.
       */

      ret = nxsem_tickwait_uninterruptible(&priv->xfrdone,
                                           USEC2TICK(passus * 4) + 2);
      if (ret < 0)
        {
          mibspi_putreg(priv, RM57_MIBSPI_TGITENCR_OFFSET,
                        MIBSPI_TGINT_RDY(0));
          nxsem_reset(&priv->xfrdone, 0);
        }
    }
  else
    {
      ret = -ETIMEDOUT;
      for (idle = 0; idle < RM57_MIBSPI_POLL_LIMIT; idle++)
        {
          if ((mibspi_getreg(priv, RM57_MIBSPI_TGINTFLG_OFFSET) &
               MIBSPI_TGINT_RDY(0)) != 0)
            {
              mibspi_putreg(priv, RM57_MIBSPI_TGINTFLG_OFFSET,
                            MIBSPI_TGINT_RDY(0));
              ret = OK;
              break;
            }
        }
    }

  if (ret < 0)
    {
      spierr("ERROR: MibSPI%d: %zu-word pass timed out, TG0CTRL=%08" PRIx32
             " LTGPEND=%08" PRIx32 " FLG=%08" PRIx32 "\n",
             priv->bus, nwords,
             mibspi_getreg(priv, RM57_MIBSPI_TGCTRL_OFFSET(0)),
             mibspi_getreg(priv, RM57_MIBSPI_LTGPEND_OFFSET),
             mibspi_getreg(priv, RM57_MIBSPI_FLG_OFFSET));
      mibspi_restart(priv);
      return -ETIMEDOUT;
    }

  /* Receive side: reading the whole word returns data and status together
   * and marks the buffer empty again.  A buffer still marked empty here
   * was never written by the sequencer.
   */

  for (i = 0; i < nwords; i++)
    {
      wd       = getreg32(priv->rambase +
                          (extended ? RM57_MIBSPI_RXRAM_EXT_OFFSET(i) :
                                      RM57_MIBSPI_RXRAM_OFFSET(i)));
      *errors |= wd & (MIBSPI_BUF_ALLERRORS | MIBSPI_BUF_RXEMPTY);

      if (rxbuffer != NULL)
        {
          if (wide)
            {
              rx16[i] = wd & MIBSPI_BUF_RXDATA_MASK;
            }
          else
            {
              rx8[i] = wd & 0xff;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: mibspi_mbexchange
 *
 * Description:
 *   Exchange a block of words in multi-buffer mode, as many TG0 passes as
 *   it takes.  The chip select is the board's GIO, so it stays asserted
 *   across the gap between passes.
 *
 ****************************************************************************/

static void mibspi_mbexchange(struct rm57_mibspi_s *priv,
                              const void *txbuffer, void *rxbuffer,
                              size_t nwords)
{
  size_t maxpass = MIN(priv->nbuf, 255);
  uint32_t errors = 0;
  size_t offset;
  size_t n;

  for (offset = 0; offset < nwords; offset += n)
    {
      n = MIN(nwords - offset, maxpass);
      if (mibspi_mbpass(priv, txbuffer, rxbuffer, offset, n, &errors) < 0)
        {
          return;
        }
    }

  if (errors != 0)
    {
      spierr("ERROR: MibSPI%d: %zu words, receive status %08" PRIx32 "\n",
             priv->bus, nwords, errors);
    }
}
#endif /* CONFIG_RM57_MIBSPI_MULTIBUF */

#ifdef CONFIG_RM57_MIBSPI_DMA
/****************************************************************************
 * Name: mibspi_dmacallback
 *
 * Description:
 *   Receive channel block complete: every word has been shifted and read
 *   back, so the transfer is over.
 *
 ****************************************************************************/

static void mibspi_dmacallback(DMA_HANDLE handle, uint8_t status, void *arg)
{
  struct rm57_mibspi_s *priv = arg;

  if ((status & RM57_DMA_STATUS_BTC) != 0)
    {
      nxsem_post(&priv->xfrdone);
    }
}

/****************************************************************************
 * Name: mibspi_dmainitialize
 *
 * Description:
 *   Claim the two DMA channels of a module configured for DMA.  Without
 *   them the module simply stays on the polled path.
 *
 ****************************************************************************/

static void mibspi_dmainitialize(struct rm57_mibspi_s *priv)
{
  if (priv->txbuf == NULL)
    {
      return;
    }

  priv->txdma = rm57_dmachannel(priv->txreq);
  priv->rxdma = rm57_dmachannel(priv->rxreq);

  if (priv->txdma == NULL || priv->rxdma == NULL)
    {
      spierr("ERROR: MibSPI%d: no DMA channels, staying polled\n",
             priv->bus);

      if (priv->txdma != NULL)
        {
          rm57_dmafree(priv->txdma);
          priv->txdma = NULL;
        }

      if (priv->rxdma != NULL)
        {
          rm57_dmafree(priv->rxdma);
          priv->rxdma = NULL;
        }

      return;
    }

  spiinfo("MibSPI%d: DMA on request lines %u (TX) and %u (RX)\n",
          priv->bus, priv->txreq, priv->rxreq);
}

/****************************************************************************
 * Name: mibspi_dmablock
 *
 * Description:
 *   Exchange up to one bounce buffer's worth of words by DMA.
 *
 *   The transmit channel writes the data half of SPIDAT1 only, so every
 *   word goes out with the control field mibspi_latchformat() left there.
 *   The module raises a transmit request whenever a word moves into the
 *   shift register, the first one as DMAREQEN is set, and a receive
 *   request whenever a word lands in SPIBUF (SPNU562A section 27.2.3.1);
 *   both channels move one halfword per request.
 *
 * Returned Value:
 *   Zero on success; -ETIMEDOUT if the receive channel did not complete,
 *   in which case the module has been restarted.
 *
 ****************************************************************************/

static int mibspi_dmablock(struct rm57_mibspi_s *priv,
                           const void *txbuffer, void *rxbuffer,
                           size_t offset, size_t nwords)
{
  const uint8_t *tx8 = (const uint8_t *)txbuffer + offset;
  const uint16_t *tx16 = (const uint16_t *)txbuffer + offset;
  uint8_t *rx8 = (uint8_t *)rxbuffer + offset;
  uint16_t *rx16 = (uint16_t *)rxbuffer + offset;
  uintptr_t txbuf = (uintptr_t)priv->txbuf;
  uintptr_t rxbuf = (uintptr_t)priv->rxbuf;
  struct rm57_dmacfg_s cfg;
  bool wide = priv->nbits > 8;
  uint32_t blockus;
  uint32_t errors;
  size_t i;
  int ret;

  for (i = 0; i < nwords; i++)
    {
      if (txbuffer == NULL)
        {
          priv->txbuf[i] = 0xffff;
        }
      else
        {
          priv->txbuf[i] = wide ? tx16[i] : tx8[i];
        }
    }

  /* The transmit data out to memory; nothing of the receive buffer left
   * in the cache to be written back over what the DMA puts there.
   */

  up_clean_dcache(txbuf, txbuf + RM57_MIBSPI_DMAWORDS * sizeof(uint16_t));
  up_invalidate_dcache(rxbuf,
                       rxbuf + RM57_MIBSPI_DMAWORDS * sizeof(uint16_t));

  cfg.saddr     = priv->base + RM57_MIBSPI_BUF_OFFSET;
  cfg.daddr     = rxbuf;
  cfg.nframes   = nwords;
  cfg.nelems    = 1;
  cfg.chctrl    = DMA_CHCTRL_RES_16BIT | DMA_CHCTRL_WES_16BIT |
                  DMA_CHCTRL_ADDMR_CONST | DMA_CHCTRL_ADDMW_INCR;
  cfg.parassign = DMA_PAR_AB_B_RD_A_WR;
  rm57_dmasetup(priv->rxdma, &cfg);

  cfg.saddr     = txbuf;
  cfg.daddr     = priv->base + RM57_MIBSPI_DAT1_OFFSET;
  cfg.chctrl    = DMA_CHCTRL_RES_16BIT | DMA_CHCTRL_WES_16BIT |
                  DMA_CHCTRL_ADDMR_INCR | DMA_CHCTRL_ADDMW_CONST;
  cfg.parassign = DMA_PAR_AB_A_RD_B_WR;
  rm57_dmasetup(priv->txdma, &cfg);

  mibspi_drain(priv);

  rm57_dmastart(priv->rxdma, mibspi_dmacallback, priv, false);
  rm57_dmastart(priv->txdma, NULL, NULL, false);
  mibspi_putreg(priv, RM57_MIBSPI_INT0_OFFSET, MIBSPI_INT0_DMAREQEN);

  /* Four times the nominal block time, plus two ticks of slack */

  blockus = (uint32_t)(((uint64_t)nwords * priv->nbits * USEC_PER_SEC +
                        priv->actual - 1) / priv->actual);
  ret = nxsem_tickwait_uninterruptible(&priv->xfrdone,
                                       USEC2TICK(blockus * 4) + 2);

  mibspi_putreg(priv, RM57_MIBSPI_INT0_OFFSET, 0);
  rm57_dmastop(priv->txdma);
  rm57_dmastop(priv->rxdma);

  if (ret < 0)
    {
      nxsem_reset(&priv->xfrdone, 0);
      spierr("ERROR: MibSPI%d: %zu-word DMA timed out, FLG=%08" PRIx32
             " residual TX %zu RX %zu\n",
             priv->bus, nwords,
             mibspi_getreg(priv, RM57_MIBSPI_FLG_OFFSET),
             rm57_dmaresidual(priv->txdma), rm57_dmaresidual(priv->rxdma));
      mibspi_restart(priv);
      return -ETIMEDOUT;
    }

  /* Drop anything the cache may have picked up while the DMA ran */

  up_invalidate_dcache(rxbuf,
                       rxbuf + RM57_MIBSPI_DMAWORDS * sizeof(uint16_t));

  if (rxbuffer != NULL)
    {
      for (i = 0; i < nwords; i++)
        {
          if (wide)
            {
              rx16[i] = priv->rxbuf[i];
            }
          else
            {
              rx8[i] = (uint8_t)priv->rxbuf[i];
            }
        }
    }

  /* The DMA reads only the data half of SPIBUF, so the error flags are
   * taken from SPIFLG instead.
   */

  errors  = mibspi_getreg(priv, RM57_MIBSPI_FLG_OFFSET);
  errors &= MIBSPI_INT_ALLERRORS;
  if (errors != 0)
    {
      spierr("ERROR: MibSPI%d: %zu-word DMA, error flags %08" PRIx32 "\n",
             priv->bus, nwords, errors);
      mibspi_putreg(priv, RM57_MIBSPI_FLG_OFFSET, errors);
    }

  return OK;
}

/****************************************************************************
 * Name: mibspi_dmaexchange
 *
 * Description:
 *   Exchange a block of words by DMA, one bounce buffer at a time.
 *
 ****************************************************************************/

static void mibspi_dmaexchange(struct rm57_mibspi_s *priv,
                               const void *txbuffer, void *rxbuffer,
                               size_t nwords)
{
  size_t offset;
  size_t n;

  for (offset = 0; offset < nwords; offset += n)
    {
      n = MIN(nwords - offset, RM57_MIBSPI_DMAWORDS);
      if (mibspi_dmablock(priv, txbuffer, rxbuffer, offset, n) < 0)
        {
          return;
        }
    }
}
#endif /* CONFIG_RM57_MIBSPI_DMA */

/****************************************************************************
 * Name: mibspi_hwinitialize
 *
 * Description:
 *   Bring the module out of reset as a 3-pin master.
 *
 ****************************************************************************/

static void mibspi_hwinitialize(struct rm57_mibspi_s *priv)
{
  uint32_t csmask = (1 << priv->ncs) - 1;
  uint32_t regval;

  /* Pulse the local reset: every control register returns to its default,
   * the pin control registers included.
   */

  mibspi_putreg(priv, RM57_MIBSPI_GCR0_OFFSET, 0);
  mibspi_putreg(priv, RM57_MIBSPI_GCR0_OFFSET, MIBSPI_GCR0_NRESET);

#ifdef CONFIG_RM57_MIBSPI_MULTIBUF
  /* Multi-buffer mode is enabled first thing after reset (SPNU562A Table
   * 27-35).  Leaving reset also starts the automatic initialization of the
   * buffer RAM, which resets the multi-buffer registers as it goes, so
   * nothing in that range is touched until it is over.  The extended
   * buffer key is ignored by the modules that do not have the feature.
   */

  if (priv->multibuf)
    {
      uint32_t i;

      mibspi_putreg(priv, RM57_MIBSPI_MIBSPIE_OFFSET,
                    MIBSPI_MIBSPIE_MSPIENA |
                    (priv->nbuf > RM57_MIBSPI_NBUF ?
                     MIBSPI_MIBSPIE_EXTBUF_ENABLE :
                     MIBSPI_MIBSPIE_EXTBUF_DISABLE));

      for (i = 0; i < RM57_MIBSPI_POLL_LIMIT; i++)
        {
          if ((mibspi_getreg(priv, RM57_MIBSPI_FLG_OFFSET) &
               MIBSPI_FLG_BUFINITACTIVE) == 0)
            {
              break;
            }
        }

      if (i >= RM57_MIBSPI_POLL_LIMIT)
        {
          spierr("ERROR: MibSPI%d: buffer RAM initialization stuck\n",
                 priv->bus);
        }
    }
  else
#endif
    {
      /* A module in compatibility mode has nothing past PMCTRL */

      priv->nbuf = 0;
    }

  /* Master, clocking SPICLK itself */

  regval = MIBSPI_GCR1_MASTER | MIBSPI_GCR1_CLKMOD;
#ifdef CONFIG_RM57_MIBSPI_LOOPBACK
  regval |= MIBSPI_GCR1_LOOPBACK;
#endif
  mibspi_putreg(priv, RM57_MIBSPI_GCR1_OFFSET, regval);

  /* No interrupts, no DMA requests; nothing to wait for on SPIENA */

  mibspi_putreg(priv, RM57_MIBSPI_INT0_OFFSET, 0);
  mibspi_putreg(priv, RM57_MIBSPI_LVL_OFFSET, 0);
  mibspi_putreg(priv, RM57_MIBSPI_DELAY_OFFSET, 0);
  mibspi_putreg(priv, RM57_MIBSPI_DEF_OFFSET, MIBSPI_DEF_CSDEF_MASK);

  /* Pins.  The chip selects and SPIENA start out as GIO inputs with
   * pull-ups, so an unclaimed chip select idles deselected; SOMI is pulled
   * up as well so that a bus with nothing driving it reads all ones.
   * SPICLK, SIMO and SOMI become functional last - their direction then
   * follows GCR1.MASTER.
   */

  mibspi_putreg(priv, RM57_MIBSPI_PC1_OFFSET, 0);
  mibspi_putreg(priv, RM57_MIBSPI_PC4_OFFSET, csmask);
  mibspi_putreg(priv, RM57_MIBSPI_PC6_OFFSET, 0);
  mibspi_putreg(priv, RM57_MIBSPI_PC7_OFFSET, 0);
  mibspi_putreg(priv, RM57_MIBSPI_PC8_OFFSET,
                csmask | MIBSPI_PC_ENA | RM57_MIBSPI_FUNCPINS);
  mibspi_putreg(priv, RM57_MIBSPI_PC0_OFFSET, RM57_MIBSPI_FUNCPINS);

  /* Power-up format.  SPIEN goes on only after everything else is
   * configured (SPNU562A section 27.2.11.1).
   */

  priv->frequency = RM57_MIBSPI_DEFFREQ;
  priv->mode      = SPIDEV_MODE0;
  priv->nbits     = RM57_MIBSPI_DEFBITS;
  priv->lsbfirst  = false;

  mibspi_putreg(priv, RM57_MIBSPI_GCR1_OFFSET, regval | MIBSPI_GCR1_SPIEN);
  mibspi_setformat(priv);

  spiinfo("MibSPI%d: GCR1=%08" PRIx32 " PC0=%08" PRIx32 " PC1=%08" PRIx32
          " PC8=%08" PRIx32 " FLG=%08" PRIx32 "\n",
          priv->bus,
          mibspi_getreg(priv, RM57_MIBSPI_GCR1_OFFSET),
          mibspi_getreg(priv, RM57_MIBSPI_PC0_OFFSET),
          mibspi_getreg(priv, RM57_MIBSPI_PC1_OFFSET),
          mibspi_getreg(priv, RM57_MIBSPI_PC8_OFFSET),
          mibspi_getreg(priv, RM57_MIBSPI_FLG_OFFSET));

#ifdef CONFIG_RM57_MIBSPI_MULTIBUF
  if (priv->multibuf)
    {
      mibspi_mbinitialize(priv);

      spiinfo("MibSPI%d: multi-buffer, %u buffers, MIBSPIE=%08" PRIx32
              " LTGPEND=%08" PRIx32 " TG0CTRL=%08" PRIx32 "\n",
              priv->bus, priv->nbuf,
              mibspi_getreg(priv, RM57_MIBSPI_MIBSPIE_OFFSET),
              mibspi_getreg(priv, RM57_MIBSPI_LTGPEND_OFFSET),
              mibspi_getreg(priv, RM57_MIBSPI_TGCTRL_OFFSET(0)));
    }
#endif

#ifdef CONFIG_RM57_MIBSPI_DMA
  mibspi_dmainitialize(priv);
#endif
}

/****************************************************************************
 * Name: mibspi_instance
 *
 * Description:
 *   Map a module number to its state, initializing the hardware on first
 *   use.
 *
 ****************************************************************************/

static struct rm57_mibspi_s *mibspi_instance(int bus)
{
  struct rm57_mibspi_s *priv;

  switch (bus)
    {
#ifdef CONFIG_RM57_MIBSPI1
      case 1:
        priv = &g_mibspi1dev;
        break;
#endif

#ifdef CONFIG_RM57_MIBSPI2
      case 2:
        priv = &g_mibspi2dev;
        break;
#endif

#ifdef CONFIG_RM57_MIBSPI3
      case 3:
        priv = &g_mibspi3dev;
        break;
#endif

#ifdef CONFIG_RM57_MIBSPI4
      case 4:
        priv = &g_mibspi4dev;
        break;
#endif

#ifdef CONFIG_RM57_MIBSPI5
      case 5:
        priv = &g_mibspi5dev;
        break;
#endif

      default:
        return NULL;
    }

  nxmutex_lock(&g_mibspi_initlock);
  if (!priv->initialized)
    {
      mibspi_hwinitialize(priv);
      priv->initialized = true;
    }

  nxmutex_unlock(&g_mibspi_initlock);
  return priv;
}

/****************************************************************************
 * Name: mibspi_lock
 ****************************************************************************/

static int mibspi_lock(struct spi_dev_s *dev, bool lock)
{
  struct rm57_mibspi_s *priv = (struct rm57_mibspi_s *)dev;

  if (lock)
    {
      return nxmutex_lock(&priv->lock);
    }

  return nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: mibspi_setfrequency
 ****************************************************************************/

static uint32_t mibspi_setfrequency(struct spi_dev_s *dev,
                                    uint32_t frequency)
{
  struct rm57_mibspi_s *priv = (struct rm57_mibspi_s *)dev;

  if (frequency != priv->frequency)
    {
      priv->frequency = frequency;
      mibspi_setformat(priv);
    }

  return priv->actual;
}

/****************************************************************************
 * Name: mibspi_setmode
 ****************************************************************************/

static void mibspi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct rm57_mibspi_s *priv = (struct rm57_mibspi_s *)dev;

  if (mode > SPIDEV_MODE3)
    {
      spierr("ERROR: MibSPI%d: unsupported mode %d\n", priv->bus, mode);
      return;
    }

  if (mode != priv->mode)
    {
      priv->mode = mode;
      mibspi_setformat(priv);
    }
}

/****************************************************************************
 * Name: mibspi_setbits
 ****************************************************************************/

static void mibspi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct rm57_mibspi_s *priv = (struct rm57_mibspi_s *)dev;

  if (nbits < MIBSPI_FMT_CHARLEN_MIN || nbits > MIBSPI_FMT_CHARLEN_MAX)
    {
      spierr("ERROR: MibSPI%d: unsupported word length %d\n",
             priv->bus, nbits);
      return;
    }

  if (nbits != priv->nbits)
    {
      priv->nbits = nbits;
      mibspi_setformat(priv);
    }
}

/****************************************************************************
 * Name: mibspi_hwfeatures
 ****************************************************************************/

#ifdef CONFIG_SPI_HWFEATURES
static int mibspi_hwfeatures(struct spi_dev_s *dev,
                             spi_hwfeatures_t features)
{
#ifdef CONFIG_SPI_BITORDER
  struct rm57_mibspi_s *priv = (struct rm57_mibspi_s *)dev;
  bool lsbfirst = (features & HWFEAT_LSBFIRST) != 0;

  if (lsbfirst != priv->lsbfirst)
    {
      priv->lsbfirst = lsbfirst;
      mibspi_setformat(priv);
    }

  features &= ~HWFEAT_LSBFIRST;
#endif

  return features == 0 ? OK : -ENOSYS;
}
#endif

/****************************************************************************
 * Name: mibspi_cmexchange
 *
 * Description:
 *   Exchange a block of words in compatibility mode, polled.
 *
 *   A second word is loaded while the first is still shifting so that
 *   SPICLK runs without gaps, but never a third: SPIFLG.TXINTFLG says the
 *   transmit buffer has room, and capping the words in flight at what the
 *   receive side can hold makes an overrun impossible however late the
 *   loop is in getting back to SPIBUF.
 *
 ****************************************************************************/

static void mibspi_cmexchange(struct rm57_mibspi_s *priv,
                              const void *txbuffer, void *rxbuffer,
                              size_t nwords)
{
  const uint8_t *tx8 = txbuffer;
  const uint16_t *tx16 = txbuffer;
  uint8_t *rx8 = rxbuffer;
  uint16_t *rx16 = rxbuffer;
  bool wide = priv->nbits > 8;
  uint32_t errors = 0;
  uint32_t idle = 0;
  uint32_t flg;
  uint32_t buf;
  uint32_t wd;
  size_t sent = 0;
  size_t rcvd = 0;

  mibspi_drain(priv);

  while (rcvd < nwords)
    {
      flg = mibspi_getreg(priv, RM57_MIBSPI_FLG_OFFSET);

      /* The first word of a burst goes out unconditionally; TXINTFLG is
       * only meaningful once something has been written.
       */

      if (sent < nwords &&
          (sent == rcvd ||
           (sent - rcvd < RM57_MIBSPI_PIPELINE &&
            (flg & MIBSPI_INT_TX) != 0)))
        {
          if (txbuffer == NULL)
            {
              wd = 0xffff;
            }
          else if (wide)
            {
              wd = tx16[sent];
            }
          else
            {
              wd = tx8[sent];
            }

          mibspi_putreg(priv, RM57_MIBSPI_DAT1_OFFSET,
                        RM57_MIBSPI_DAT1_CTRL |
                        (wd & MIBSPI_DAT1_TXDATA_MASK));
          sent++;
          idle = 0;
        }

      if ((flg & MIBSPI_INT_RX) != 0)
        {
          buf     = mibspi_getreg(priv, RM57_MIBSPI_BUF_OFFSET);
          errors |= buf & MIBSPI_BUF_ALLERRORS;

          if (rxbuffer != NULL)
            {
              if (wide)
                {
                  rx16[rcvd] = buf & MIBSPI_BUF_RXDATA_MASK;
                }
              else
                {
                  rx8[rcvd] = buf & 0xff;
                }
            }

          rcvd++;
          idle = 0;
        }
      else if (++idle > RM57_MIBSPI_POLL_LIMIT)
        {
          spierr("ERROR: MibSPI%d: stalled at %zu/%zu sent %zu "
                 "FLG=%08" PRIx32 "\n",
                 priv->bus, rcvd, nwords, sent, flg);
          mibspi_restart(priv);
          return;
        }
    }

  if (errors != 0)
    {
      spierr("ERROR: MibSPI%d: %zu words, error flags %08" PRIx32 "\n",
             priv->bus, nwords, errors);
      mibspi_drain(priv);
    }
}

/****************************************************************************
 * Name: mibspi_exchange
 *
 * Description:
 *   Exchange a block of words.  Words are 8 bits wide in the buffers when
 *   the word length is 8 or less and 16 bits otherwise; either buffer may
 *   be NULL.
 *
 ****************************************************************************/

static void mibspi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                            void *rxbuffer, size_t nwords)
{
  struct rm57_mibspi_s *priv = (struct rm57_mibspi_s *)dev;

#ifdef CONFIG_RM57_MIBSPI_MULTIBUF
  if (priv->multibuf)
    {
      mibspi_mbexchange(priv, txbuffer, rxbuffer, nwords);
      return;
    }
#endif

#ifdef CONFIG_RM57_MIBSPI_DMA
  /* DMA where it pays and where the caller can sleep */

  if (priv->rxdma != NULL && nwords >= CONFIG_RM57_MIBSPI_DMATHRESHOLD &&
      OSINIT_OS_READY() && !up_interrupt_context())
    {
      mibspi_dmaexchange(priv, txbuffer, rxbuffer, nwords);
      return;
    }
#endif

  mibspi_cmexchange(priv, txbuffer, rxbuffer, nwords);
}

/****************************************************************************
 * Name: mibspi_send
 ****************************************************************************/

static uint32_t mibspi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct rm57_mibspi_s *priv = (struct rm57_mibspi_s *)dev;
  uint16_t tx16 = wd;
  uint16_t rx16 = 0xffff;
  uint8_t tx8 = wd;
  uint8_t rx8 = 0xff;

  if (priv->nbits > 8)
    {
      mibspi_exchange(dev, &tx16, &rx16, 1);
      return rx16;
    }

  mibspi_exchange(dev, &tx8, &rx8, 1);
  return rx8;
}

/****************************************************************************
 * Name: mibspi_sndblock and mibspi_recvblock
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void mibspi_sndblock(struct spi_dev_s *dev, const void *buffer,
                            size_t nwords)
{
  mibspi_exchange(dev, buffer, NULL, nwords);
}

static void mibspi_recvblock(struct spi_dev_s *dev, void *buffer,
                             size_t nwords)
{
  mibspi_exchange(dev, NULL, buffer, nwords);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_mibspibus_initialize
 ****************************************************************************/

struct spi_dev_s *rm57_mibspibus_initialize(int bus)
{
  struct rm57_mibspi_s *priv = mibspi_instance(bus);

  if (priv == NULL)
    {
      spierr("ERROR: MibSPI%d is not available\n", bus);
      return NULL;
    }

  return &priv->spidev;
}

/****************************************************************************
 * Name: rm57_mibspi_csconfig
 ****************************************************************************/

int rm57_mibspi_csconfig(int bus, uint8_t csmask)
{
  struct rm57_mibspi_s *priv = mibspi_instance(bus);
  uint32_t regval;

  if (priv == NULL)
    {
      return -ENODEV;
    }

  if ((csmask >> priv->ncs) != 0)
    {
      return -EINVAL;
    }

  /* Latch high first, then turn the pins round, so they never drive low
   * on the way to becoming outputs.
   */

  nxmutex_lock(&g_mibspi_initlock);

  mibspi_putreg(priv, RM57_MIBSPI_PC4_OFFSET, csmask);

  regval = mibspi_getreg(priv, RM57_MIBSPI_PC0_OFFSET);
  mibspi_putreg(priv, RM57_MIBSPI_PC0_OFFSET, regval & ~(uint32_t)csmask);

  regval = mibspi_getreg(priv, RM57_MIBSPI_PC1_OFFSET);
  mibspi_putreg(priv, RM57_MIBSPI_PC1_OFFSET, regval | csmask);

  nxmutex_unlock(&g_mibspi_initlock);

  spiinfo("MibSPI%d: CS mask %02x PC1=%08" PRIx32 " PC2=%08" PRIx32 "\n",
          bus, csmask, mibspi_getreg(priv, RM57_MIBSPI_PC1_OFFSET),
          mibspi_getreg(priv, RM57_MIBSPI_PC2_OFFSET));
  return OK;
}

/****************************************************************************
 * Name: rm57_mibspi_cswrite
 ****************************************************************************/

void rm57_mibspi_cswrite(int bus, int cs, bool level)
{
  uint32_t base;

  switch (bus)
    {
#ifdef CONFIG_RM57_MIBSPI1
      case 1:
        base = RM57_MIBSPI1_BASE;
        break;
#endif

#ifdef CONFIG_RM57_MIBSPI2
      case 2:
        base = RM57_MIBSPI2_BASE;
        break;
#endif

#ifdef CONFIG_RM57_MIBSPI3
      case 3:
        base = RM57_MIBSPI3_BASE;
        break;
#endif

#ifdef CONFIG_RM57_MIBSPI4
      case 4:
        base = RM57_MIBSPI4_BASE;
        break;
#endif

#ifdef CONFIG_RM57_MIBSPI5
      case 5:
        base = RM57_MIBSPI5_BASE;
        break;
#endif

      default:
        return;
    }

  if (cs < 0 || cs >= RM57_NMIBSPI_CS)
    {
      return;
    }

  putreg32(MIBSPI_PC_SCS(cs),
           base + (level ? RM57_MIBSPI_PC4_OFFSET : RM57_MIBSPI_PC5_OFFSET));
}

#endif /* CONFIG_RM57_MIBSPI */

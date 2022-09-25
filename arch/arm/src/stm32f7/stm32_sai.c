/****************************************************************************
 * arch/arm/src/stm32f7/stm32_sai.c
 *
 *   Copyright (C) 2013-2014, 2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *   Copyright (c) 2016 Motorola Mobility, LLC. All rights reserved.
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
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/queue.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_dma.h"
#include "stm32_gpio.h"
#include "stm32_sai.h"
#include "stm32_pwr.h"

#ifdef CONFIG_STM32F7_SAI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#endif

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO required by this driver
#endif

#ifndef CONFIG_I2S
#  error CONFIG_I2S required by this driver
#endif

#ifdef CONFIG_STM32F7_SAI_POLLING
#  error "Polling SAI not yet supported"
#endif

#ifdef CONFIG_STM32F7_SAI_INTERRUPTS
#  error "Interrupt driven SAI not yet supported"
#endif

#ifndef CONFIG_STM32F7_SAI_DEFAULT_SAMPLERATE
#  define CONFIG_STM32F7_SAI_DEFAULT_SAMPLERATE  (48000)
#endif

#ifndef CONFIG_STM32F7_SAI_DEFAULT_DATALEN
#  define CONFIG_STM32F7_SAI_DEFAULT_DATALEN     (16)
#endif

#ifndef CONFIG_STM32F7_SAI_MAXINFLIGHT
#  define CONFIG_STM32F7_SAI_MAXINFLIGHT         (16)
#endif

#ifdef CONFIG_STM32F7_SAI1
#ifndef STM32F7_SAI1_FREQUENCY
#  error "Please define STM32F7_SAI1_FREQUENCY in board.h"
#endif
#endif

#ifdef CONFIG_STM32F7_SAI2
#ifndef STM32F7_SAI2_FREQUENCY
#  error "Please define STM32F7_SAI1_FREQUENCY in board.h"
#endif
#endif

#ifdef CONFIG_STM32F7_SAI_DMA
/* SAI DMA priority */

#  if defined(CONFIG_STM32F7_SAI_DMAPRIO)
#    define SAI_DMA_PRIO       CONFIG_STM32F7_SAI_DMAPRIO
#  else
#    define SAI_DMA_PRIO       DMA_SCR_PRIVERYHI
#  endif

#  if (SAI_DMA_PRIO & ~DMA_SCR_PL_MASK) != 0
#    error "Illegal value for CONFIG_STM32F7_SAI_DMAPRIO"
#  endif

/* DMA channel/stream configuration register settings.  The following
 * must be selected.  The DMA driver will select the remaining fields.
 *
 * - 32-bit DMA
 * - Memory increment
 * - Direction (memory-to-peripheral, peripheral-to-memory)
 * - Memory burst size (F4 only)
 */

/* STM32 stream configuration register (SCR) settings base settings sans
 * priority.
 */

#  define SAI_RXDMA8_CONFIG   (DMA_SCR_PFCTRL | DMA_SCR_DIR_P2M|DMA_SCR_MINC | \
                                  DMA_SCR_PSIZE_8BITS | DMA_SCR_MSIZE_8BITS | \
                                  DMA_SCR_PBURST_INCR4 | DMA_SCR_MBURST_INCR4)
#  define SAI_RXDMA16_CONFIG   (DMA_SCR_PFCTRL | DMA_SCR_DIR_P2M|DMA_SCR_MINC | \
                                  DMA_SCR_PSIZE_16BITS | DMA_SCR_MSIZE_16BITS | \
                                  DMA_SCR_PBURST_INCR4 | DMA_SCR_MBURST_INCR4)
#  define SAI_RXDMA32_CONFIG   (DMA_SCR_PFCTRL | DMA_SCR_DIR_P2M|DMA_SCR_MINC | \
                                  DMA_SCR_PSIZE_32BITS | DMA_SCR_MSIZE_32BITS | \
                                  DMA_SCR_PBURST_INCR4 | DMA_SCR_MBURST_INCR4)

#  define SAI_TXDMA8_CONFIG    (DMA_SCR_DIR_M2P | DMA_SCR_MINC | \
                                  DMA_SCR_PSIZE_8BITS | DMA_SCR_MSIZE_8BITS | \
                                  DMA_SCR_PBURST_INCR4 | DMA_SCR_MBURST_INCR4)
#  define SAI_TXDMA16_CONFIG   (DMA_SCR_DIR_M2P | DMA_SCR_MINC | \
                                  DMA_SCR_PSIZE_16BITS | DMA_SCR_MSIZE_16BITS | \
                                  DMA_SCR_PBURST_INCR4 | DMA_SCR_MBURST_INCR4)
#  define SAI_TXDMA32_CONFIG   (DMA_SCR_DIR_M2P | DMA_SCR_MINC | \
                                  DMA_SCR_PSIZE_32BITS | DMA_SCR_MSIZE_32BITS | \
                                  DMA_SCR_PBURST_INCR4 | DMA_SCR_MBURST_INCR4)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2S buffer container */

struct sai_buffer_s
{
  struct sai_buffer_s *flink;  /* Supports a singly linked list */
  i2s_callback_t callback;     /* Function to call when the transfer completes */
  uint32_t timeout;            /* The timeout value to use with transfers */
  void *arg;                   /* The argument to be returned with the callback */
  struct ap_buffer_s *apb;     /* The audio buffer */
  int result;                  /* The result of the transfer */
};

/* The state of the one SAI peripheral */

struct stm32f7_sai_s
{
  struct i2s_dev_s dev;        /* Externally visible I2S interface */

  /* Callback for changes in sample rate */

  stm32_sai_sampleratecb_t sampleratecb;
  uintptr_t base;              /* SAI block register base address */
  sem_t exclsem;               /* Assures mutually exclusive access to SAI */
  uint32_t frequency;          /* SAI clock frequency */
  uint32_t syncen;             /* Synchronization setting */
#ifdef CONFIG_STM32F7_SAI_DMA
  uint16_t dma_ch;             /* DMA channel number */
  DMA_HANDLE dma;              /* DMA channel handle */
  uint32_t dma_ccr;            /* DMA control register */
#endif
  uint8_t datalen;             /* Data width */
  uint32_t samplerate;         /* Data sample rate */
  uint8_t rxenab:1;            /* True: RX transfers enabled */
  uint8_t txenab:1;            /* True: TX transfers enabled */
  struct wdog_s dog;           /* Watchdog that handles timeouts */
  sq_queue_t pend;             /* A queue of pending transfers */
  sq_queue_t act;              /* A queue of active transfers */
  sq_queue_t done;             /* A queue of completed transfers */
  struct work_s work;          /* Supports worker thread operations */

  /* Pre-allocated pool of buffer containers */

  sem_t bufsem;                   /* Buffer wait semaphore */
  struct sai_buffer_s *freelist;  /* A list a free buffer containers */
  struct sai_buffer_s containers[CONFIG_STM32F7_SAI_MAXINFLIGHT];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_DEBUG_I2S_INFO
static void     sai_dump_regs(struct stm32f7_sai_s *priv, const char *msg);
#else
#  define       sai_dump_regs(s,m)
#endif

/* Semaphore helpers */

static void     sai_exclsem_take(struct stm32f7_sai_s *priv);
#define         sai_exclsem_give(priv) nxsem_post(&priv->exclsem)

static void     sai_bufsem_take(struct stm32f7_sai_s *priv);
#define         sai_bufsem_give(priv) nxsem_post(&priv->bufsem)

/* Buffer container helpers */

static struct sai_buffer_s *
                sai_buf_allocate(struct stm32f7_sai_s *priv);
static void     sai_buf_free(struct stm32f7_sai_s *priv,
                  struct sai_buffer_s *bfcontainer);
static void     sai_buf_initialize(struct stm32f7_sai_s *priv);

/* DMA support */

#ifdef CONFIG_STM32F7_SAI_DMA
static void     sai_schedule(struct stm32f7_sai_s *priv, int result);
static void     sai_dma_callback(DMA_HANDLE handle, uint8_t isr, void *arg);
#endif

/* I2S methods */

static uint32_t sai_samplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t sai_datawidth(struct i2s_dev_s *dev, int bits);
static int      sai_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                  i2s_callback_t callback, void *arg, uint32_t timeout);
static int      sai_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                  i2s_callback_t callback, void *arg,
                  uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2S device operations */

static const struct i2s_ops_s g_i2sops =
{
  /* Receiver methods */

  .i2s_rxsamplerate = sai_samplerate,
  .i2s_rxdatawidth  = sai_datawidth,
  .i2s_receive      = sai_receive,

  /* Transmitter methods */

  .i2s_txsamplerate = sai_samplerate,
  .i2s_txdatawidth  = sai_datawidth,
  .i2s_send         = sai_send,
};

/* SAI1 state */

#ifdef CONFIG_STM32F7_SAI1_A
static struct stm32f7_sai_s g_sai1a_priv =
{
  .dev.ops     = &g_i2sops,
  .base        = STM32F7_SAI1_A_BASE,
  .frequency   = STM32F7_SAI1_FREQUENCY,
#ifdef CONFIG_STM32F7_SAI1_A_SYNC_WITH_B
  .syncen      = SAI_CR1_SYNCEN_INTERNAL,
#else
  .syncen      = SAI_CR1_SYNCEN_ASYNCH,
#endif
#ifdef CONFIG_STM32F7_SAI_DMA
  .dma_ch      = DMACHAN_SAI1_A,
#endif
  .datalen     = CONFIG_STM32F7_SAI_DEFAULT_DATALEN,
  .samplerate  = CONFIG_STM32F7_SAI_DEFAULT_SAMPLERATE,
};
#endif

#ifdef CONFIG_STM32F7_SAI1_B
static struct stm32f7_sai_s g_sai1b_priv =
{
  .dev.ops     = &g_i2sops,
  .base        = STM32F7_SAI1_B_BASE,
  .frequency   = STM32F7_SAI1_FREQUENCY,
#ifdef CONFIG_STM32F7_SAI1_B_SYNC_WITH_A
  .syncen      = SAI_CR1_SYNCEN_INTERNAL,
#else
  .syncen      = SAI_CR1_SYNCEN_ASYNCH,
#endif
#ifdef CONFIG_STM32F7_SAI_DMA
  .dma_ch      = DMACHAN_SAI1_B,
#endif
  .datalen     = CONFIG_STM32F7_SAI_DEFAULT_DATALEN,
  .samplerate  = CONFIG_STM32F7_SAI_DEFAULT_SAMPLERATE,
};
#endif

/* SAI2 state */

#ifdef CONFIG_STM32F7_SAI2_A
static struct stm32f7_sai_s g_sai2a_priv =
{
  .dev.ops     = &g_i2sops,
  .base        = STM32F7_SAI2_A_BASE,
  .frequency   = STM32F7_SAI2_FREQUENCY,
#ifdef CONFIG_STM32F7_SAI2_A_SYNC_WITH_B
  .syncen      = SAI_CR1_SYNCEN_INTERNAL,
#else
  .syncen      = SAI_CR1_SYNCEN_ASYNCH,
#endif
#ifdef CONFIG_STM32F7_SAI_DMA
  .dma_ch      = DMACHAN_SAI2_A,
#endif
  .datalen     = CONFIG_STM32F7_SAI_DEFAULT_DATALEN,
  .samplerate  = CONFIG_STM32F7_SAI_DEFAULT_SAMPLERATE,
};
#endif

#ifdef CONFIG_STM32F7_SAI2_B
static struct stm32f7_sai_s g_sai2b_priv =
{
  .dev.ops     = &g_i2sops,
  .base        = STM32F7_SAI2_B_BASE,
  .frequency   = STM32F7_SAI2_FREQUENCY,
#ifdef CONFIG_STM32F7_SAI2_B_SYNC_WITH_A
  .syncen      = SAI_CR1_SYNCEN_INTERNAL,
#else
  .syncen      = SAI_CR1_SYNCEN_ASYNCH,
#endif
#ifdef CONFIG_STM32F7_SAI_DMA
  .dma_ch      = DMACHAN_SAI2_B,
#endif
  .datalen     = CONFIG_STM32F7_SAI_DEFAULT_DATALEN,
  .samplerate  = CONFIG_STM32F7_SAI_DEFAULT_SAMPLERATE,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sai_getbitrate
 *
 * Description:
 *   Get the currently configured bitrate
 *
 * Input Parameters:
 *   priv   - private SAI device structure
 *
 * Returned Value:
 *   The current bitrate
 *
 ****************************************************************************/

static inline uint32_t sai_getbitrate(struct stm32f7_sai_s *priv)
{
  /* Calculate the bitrate in Hz */

  return priv->samplerate * priv->datalen;
}

/****************************************************************************
 * Name: sai_getreg
 *
 * Description:
 *   Get the contents of the SAI register at offset
 *
 * Input Parameters:
 *   priv   - private SAI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline uint32_t sai_getreg(struct stm32f7_sai_s *priv, uint8_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: sai_putreg
 *
 * Description:
 *   Write a 16-bit value to the SAI register at offset
 *
 * Input Parameters:
 *   priv   - private SAI device structure
 *   offset - offset to the register of interest
 *   value  - the 32-bit value to be written
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sai_putreg(struct stm32f7_sai_s *priv, uint8_t offset,
                              uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: sai_modifyreg
 *
 * Description:
 *   Clear and set bits in the SAI register at offset
 *
 * Input Parameters:
 *   priv    - private SAI device structure
 *   offset  - offset to the register of interest
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_modifyreg(struct stm32f7_sai_s *priv, uint8_t offset,
                          uint32_t clrbits, uint32_t setbits)
{
  uint32_t regval;

  regval  = sai_getreg(priv, offset);
  regval &= ~clrbits;
  regval |= setbits;
  sai_putreg(priv, offset, regval);
}

/****************************************************************************
 * Name: sai_dump_regs
 *
 * Description:
 *   Dump the contents of all SAI block registers
 *
 * Input Parameters:
 *   priv - The SAI block controller to dump
 *   msg - Message to print before the register data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_I2S_INFO
static void sai_dump_regs(struct stm32f7_sai_s *priv, const char *msg)
{
  if (msg)
      i2sinfo("%s\n", msg);

#if 0
  i2sinfo("CR1:%08x CR2:%08x  FRCR:%08x SLOTR:%08x\n",
          sai_getreg(priv, STM32F7_SAI_CR1_OFFSET),
          sai_getreg(priv, STM32F7_SAI_CR2_OFFSET),
          sai_getreg(priv, STM32F7_SAI_FRCR_OFFSET),
          sai_getreg(priv, STM32F7_SAI_SLOTR_OFFSET));
  i2sinfo(" IM:%08x  SR:%08x CLRFR:%08x\n",
          sai_getreg(priv, STM32F7_SAI_IM_OFFSET),
          sai_getreg(priv, STM32F7_SAI_SR_OFFSET),
          sai_getreg(priv, STM32F7_SAI_CLRFR_OFFSET));
#else
  /* GCR */

#ifdef CONFIG_STM32F7_SAI1
  uint32_t gcr = getreg32(STM32F7_SAI1_GCR);
  i2sinfo("GCR: *%08x = %08x\n", STM32F7_SAI1_GCR, gcr);
#else
  uint32_t gcr = getreg32(STM32F7_SAI2_GCR);
  i2sinfo("GCR: *%08x = %08x\n", STM32F7_SAI2_GCR, gcr);
#endif

  /* CR1 */

  uint32_t cr1 = sai_getreg(priv, STM32F7_SAI_CR1_OFFSET);
  i2sinfo("CR1: *%08x = %08x\n", STM32F7_SAI_CR1_OFFSET, cr1);

  uint32_t mode = (cr1 & SAI_CR1_MODE_MASK) >> SAI_CR1_MODE_SHIFT;
  const char *mode_string[] =
  { "Master transmitter",
    "Master receiver",
    "Slave transmitter",
    "Slave receiver"
  };

  i2sinfo("\t\tCR1: MODE[1:0] = %s\n", mode_string[mode]);

  uint32_t prtcfg = (cr1 & SAI_CR1_PRTCFG_MASK) >> SAI_CR1_PRTCFG_SHIFT;
  const char *prtcfg_string[] =
  { "Free protocol",
    "SPDIF protocol",
    "AC'97 protocol",
    "Reserved"
  };

  i2sinfo("\t\tCR1: PRTCFG[3:2] = %s\n", prtcfg_string[prtcfg]);

  uint32_t ds = (cr1 & SAI_CR1_DS_MASK) >> SAI_CR1_DS_SHIFT;
  const char *ds_string[] =
  { "Reserved",
    "Reserved",
    "8 Bits",
    "10 Bits",
    "16 Bits",
    "20 Bits",
    "24 Bits",
    "32 Bits"
  };

  i2sinfo("\t\tCR1: DS[7:5] = %s\n", ds_string[ds]);

  uint32_t lsbfirst = cr1 & SAI_CR1_LSBFIRST;
  i2sinfo("\t\tCR1: LSBFIRST[8] = %s\n",
          lsbfirst ? "Data are transferred with LSB first"
                   : "Data are transferred with MSB first");
  uint32_t ckstr = cr1 & SAI_CR1_CKSTR;
  i2sinfo("\t\tCR1: CKSTR[9] = %s\n", ckstr ? "SCK falling edge"
                                            : "SCK rising edge");

  uint32_t syncen = (cr1 & SAI_CR1_SYNCEN_MASK) >> SAI_CR1_SYNCEN_SHIFT;
  const char *syncen_string[] =
  { "audio sub-block in asynchronous mode",
    "audio sub-block in asynchronous with the other internal audio "
      "sub-block",
    "audio sub-block in synchronous with an external SAI embedded "
      "peripheral",
    "Reserved"
  };

  i2sinfo("\t\tCR1: SYNCEN[11:10] = %s\n", syncen_string[syncen]);

  uint32_t mono = cr1 & SAI_CR1_MONO;
  i2sinfo("\t\tCR1: MONO[12] = %s\n",
          mono ? "Mono mode"
               : "Stereo mode");
  uint32_t outdriv = cr1 & SAI_CR1_OUTDRIV;
  i2sinfo("\t\tCR1: OUTDRIV[13] = %s\n",
          outdriv ? "Audio block output driven immediately after "
                      "the setting of this bit"
                  : "Audio block output driven when SAIEN is set");
  uint32_t saien = cr1 & SAI_CR1_SAIEN;
  i2sinfo("\t\tCR1: SAIEN[16] = %s\n",
          saien ? "SAI audio block enabled"
                : "SAI audio block disabled");
  uint32_t dmaen = cr1 & SAI_CR1_DMAEN;
  i2sinfo("\t\tCR1: DMAEN[17] = %s\n",
          dmaen ? "DMA enabled"
                : "DMA disabled");
  uint32_t nodiv = cr1 & SAI_CR1_NODIV;
  i2sinfo("\t\tCR1: NODIV[19] = %s\n",
          nodiv ? "No divider used in the clock generator"
          : "Master clock generator is enabled");
  uint32_t mckdiv = (cr1 & SAI_CR1_MCKDIV_MASK) >> SAI_CR1_MCKDIV_SHIFT;
  i2sinfo("\t\tCR1: MCKDIV[23:20] = %d\n", mckdiv);

  /* CR2 */

  uint32_t cr2 = sai_getreg(priv, STM32F7_SAI_CR2_OFFSET);
  i2sinfo("CR2: *%08x = %08x\n", STM32F7_SAI_CR2_OFFSET, cr2);
  uint32_t fth = (cr2 & SAI_CR2_FTH_MASK) >> SAI_CR2_FTH_SHIFT;
  const char *fth_string[] =
  { "FIFO empty",
    "1/4 FIFO",
    "1/2 FIFO",
    "3/4 FIFO",
    "FIFO full",
    "Reserved",
    "Reserved",
    "Reserved"
  };

  i2sinfo("\t\tCR2: FTH[2:0] = %s\n", fth_string[fth]);
  uint32_t fflush = cr2 & SAI_CR2_FFLUSH;
  i2sinfo("\t\tCR2: FFLUSH[3] = %s\n",
          fflush ? "FIFO flush"
                 : "no FIFO flush");

  uint32_t tris = cr2 & SAI_CR2_TRIS;
  i2sinfo("\t\tCR2: TRIS[4] = %s\n",
          tris ? "SD output line is release (HI-Z)" :
                 "SD output line is still driven by the SAI when a slot is "
                   "inactive");
  uint32_t mute = cr2 & SAI_CR2_MUTE;
  i2sinfo("\t\tCR2: MUTE[5] = %s\n",
          mute ? "Mute mode enabled"
               : "No mute mode");

  uint32_t muteval = cr2 & SAI_CR2_MUTEVAL;
  i2sinfo("\t\tCR2: MUTEVAL[6] = %s\n",
          muteval ? "Last values are sent during the mute mode"
                  : "Bit value 0 is sent during the mute mode");

  uint32_t mutecnt = (cr2 & SAI_CR2_MUTECNT_MASK) >> SAI_CR2_MUTECNT_SHIFT;
  i2sinfo("\t\tCR2: MUTECNT[12:7] = %d\n", mutecnt);

  uint32_t cpl = cr2 & SAI_CR2_CPL;
  i2sinfo("\t\tCR2: CPL[13] = %s\n",
          cpl ? "1's complement represention"
              : "2's complement represention");
  uint32_t comp = (cr2 & SAI_CR2_COMP_MASK) >> SAI_CR2_COMP_SHIFT;
  const char *comp_string[] =
  { "No companding algorithm",
    "Reserved",
    "u-Law algorithm",
    "A-Law algorithm"
  };

  i2sinfo("\t\tCR2: COMP[15:14] = %s\n", comp_string[comp]);

  /* FRCR */

  uint32_t frcr = sai_getreg(priv, STM32F7_SAI_FRCR_OFFSET);
  i2sinfo("FRCR: *%08x = %08x\n", STM32F7_SAI_FRCR_OFFSET, frcr);

  uint32_t frl = (frcr & SAI_FRCR_FRL_MASK) >> SAI_FRCR_FRL_SHIFT;
  i2sinfo("\t\tFRCR: FRL[7:0] = %d\n", frl);

  uint32_t fsall = (frcr & SAI_FRCR_FSALL_MASK) >> SAI_FRCR_FSALL_SHIFT;
  i2sinfo("\t\tFRCR: FSALL[14:8] = %d\n", fsall + 1);

  uint32_t fsdef = frcr & SAI_FRCR_FSDEF;
  i2sinfo("\t\tFRCR: FSDEF[16] = %s\n",
          fsdef ? "FS signal is a start of frame + channel side ID"
                : "FS signal is a start frame signal");

  uint32_t fspol = frcr & SAI_FRCR_FSPOL;
  i2sinfo("\t\tFRCR: FSPOL[17] = %s\n", fspol ? "FS is active high"
                                            : "FS is active low");
  uint32_t fsoff = frcr & SAI_FRCR_FSOFF;
  i2sinfo("\t\tFRCR: FSOFF[18] = %s\n",
          fsoff ? "FS one bit before first bit of slot 0"
                : "FS on first bit of slot 0");

  /* SLOTR */

  uint32_t slotr = sai_getreg(priv, STM32F7_SAI_SLOTR_OFFSET);
  i2sinfo("SLOTR: *%08x = %08x\n", STM32F7_SAI_SLOTR_OFFSET, slotr);

  uint32_t fboff = (slotr & SAI_SLOTR_FBOFF_MASK) >> SAI_SLOTR_FBOFF_SHIFT;
  i2sinfo("\t\tSLOTR: FBOFF[4:0] = %d\n", fboff);

  uint32_t slotsz = (slotr & SAI_SLOTR_SLOTSZ_MASK) >>
                      SAI_SLOTR_SLOTSZ_SHIFT;
  const char *slotsz_string[] =
  { "Same as data size",
    "16-bit",
    "32-bit",
    "Reserved"
  };

  i2sinfo("\t\tSLOTR: SLOTSZ[7:6] = %s\n", slotsz_string[slotsz]);

  uint32_t nbslot = (slotr & SAI_SLOTR_NBSLOT_MASK) >>
                      SAI_SLOTR_NBSLOT_SHIFT;
  i2sinfo("\t\tSLOTR: NBSLOT[11:8] = %d\n", nbslot + 1);

  uint32_t sloten = (slotr & SAI_SLOTR_SLOTEN_MASK) >>
                      SAI_SLOTR_SLOTEN_SHIFT;
  i2sinfo("\t\tSLOTR: SLOTEN[31:16] = %08x\n", sloten + 1);
#endif
}
#endif

/****************************************************************************
 * Name: sai_exclsem_take
 *
 * Description:
 *   Take the exclusive access semaphore handling any exceptional conditions
 *
 * Input Parameters:
 *   priv - A reference to the SAI peripheral state
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void sai_exclsem_take(struct stm32f7_sai_s *priv)
{
  int ret;

  /* Wait until we successfully get the semaphore.  EINTR is the only
   * expected 'failure' (meaning that the wait for the semaphore was
   * interrupted by a signal).
   */

  do
    {
      ret = nxsem_wait(&priv->exclsem);
      DEBUGASSERT(ret == 0 || ret == -EINTR);
    }
  while (ret == -EINTR);
}

/****************************************************************************
 * Name: sai_mckdivider
 *
 * Description:
 *   Setup the master clock divider based on the currently selected data
 *   width and the sample rate
 *
 * Input Parameters:
 *   priv - SAI device structure (only the sample rate and frequency are
 *          needed at this point).
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_mckdivider(struct stm32f7_sai_s *priv)
{
  uint32_t mckdiv;
  uint32_t tmpval;

  DEBUGASSERT(priv && priv->samplerate > 0 && priv->frequency > 0);

  /* Configure Master Clock using the following formula:
   * MCLK_x = SAI_CK_x / (MCKDIV[3:0] * 2) with MCLK_x = 256 * FS
   * FS = SAI_CK_x / (MCKDIV[3:0] * 2) * 256
   * MCKDIV[3:0] = SAI_CK_x / FS * 512
   */

  /* (freq*10) to keep Significant digits */

  tmpval = (priv->frequency * 10) / (priv->samplerate * 2 * 256);
  mckdiv = tmpval / 10;

  /* Round result to the nearest integer */

  if ((tmpval % 10) > 8)
    {
      mckdiv += 1;
    }

  sai_modifyreg(priv, STM32F7_SAI_CR1_OFFSET, SAI_CR1_MCKDIV_MASK,
                mckdiv << SAI_CR1_MCKDIV_SHIFT);
}

/****************************************************************************
 * Name: sai_timeout
 *
 * Description:
 *   The watchdog timeout without completion of the transfer.
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

static void sai_timeout(wdparm_t arg)
{
  struct stm32f7_sai_s *priv = (struct stm32f7_sai_s *)arg;
  DEBUGASSERT(priv != NULL);

#ifdef CONFIG_STM32F7_SAI_DMA
  /* Cancel the DMA */

  stm32_dmastop(priv->dma);
#endif

  /* Then schedule completion of the transfer to occur on the worker
   * thread.
   */

  sai_schedule(priv, -ETIMEDOUT);
}

/****************************************************************************
 * Name: sai_dma_setup
 *
 * Description:
 *   Setup and initiate the next DMA transfer
 *
 * Input Parameters:
 *   priv - SAI state instance
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_SAI_DMA
static int sai_dma_setup(struct stm32f7_sai_s *priv)
{
  struct sai_buffer_s *bfcontainer;
  struct ap_buffer_s *apb;
  uintptr_t samp;
  apb_samp_t nbytes;
  size_t ntransfers = 0;
  int ret;

  /* If there is already an active transmission in progress, then bail
   * returning success.
   */

  if (!sq_empty(&priv->act))
    {
      return OK;
    }

  /* If there are no pending transfer, then bail returning success */

  if (sq_empty(&priv->pend))
    {
      priv->txenab = priv->rxenab = false;
      return OK;
    }

  /* Remove the pending transfer at the head of the pending queue. */

  bfcontainer = (struct sai_buffer_s *)sq_remfirst(&priv->pend);
  DEBUGASSERT(bfcontainer && bfcontainer->apb);

  apb = bfcontainer->apb;

  /* Get the transfer information, accounting for any data offset */

  samp = (uintptr_t)&apb->samp[apb->curbyte];

  /* Configure the DMA */

  if (priv->txenab)
    {
      nbytes = apb->nbytes - apb->curbyte;

      switch (priv->datalen)
        {
          case 8:
            priv->dma_ccr = SAI_TXDMA8_CONFIG;
            ntransfers = nbytes;
            break;

          case 16:
            priv->dma_ccr = SAI_TXDMA16_CONFIG;
            DEBUGASSERT((nbytes & 0x1) == 0);
            ntransfers = nbytes >> 1;
            break;

          case 32:
            priv->dma_ccr = SAI_TXDMA32_CONFIG;
            DEBUGASSERT((nbytes & 0x3) == 0);
            ntransfers = nbytes >> 2;
            break;
        }
    }
  else if (priv->rxenab)
    {
      nbytes = apb->nmaxbytes - apb->curbyte;

      switch (priv->datalen)
        {
          case 8:
            priv->dma_ccr = SAI_RXDMA8_CONFIG;
            ntransfers = nbytes;
            break;

          case 16:
            priv->dma_ccr = SAI_RXDMA16_CONFIG;
            DEBUGASSERT((nbytes & 0x1) == 0);
            ntransfers = nbytes >> 1;
            break;

          case 32:
            priv->dma_ccr = SAI_RXDMA32_CONFIG;
            DEBUGASSERT((nbytes & 0x3) == 0);
            ntransfers = nbytes >> 2;
            break;
        }
    }

  DEBUGASSERT(ntransfers > 0);

  stm32_dmasetup(priv->dma, priv->base + STM32F7_SAI_DR_OFFSET,
                 samp, ntransfers, priv->dma_ccr);

  /* Add the container to the list of active DMAs */

  sq_addlast((sq_entry_t *)bfcontainer, &priv->act);

  /* Start the DMA, saving the container as the current active transfer */

  stm32_dmastart(priv->dma, sai_dma_callback, priv, false);

  /* Enable the transmitter */

  sai_modifyreg(priv, STM32F7_SAI_CR1_OFFSET, 0, SAI_CR1_SAIEN);

  /* Start a watchdog to catch DMA timeouts */

  if (bfcontainer->timeout > 0)
    {
      ret = wd_start(&priv->dog, bfcontainer->timeout,
                     sai_timeout, (wdparm_t)priv);

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
 * Name: sai_worker
 *
 * Description:
 *   Transfer done worker
 *
 * Input Parameters:
 *   arg - the SAI device instance cast to void*
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_worker(void *arg)
{
  struct stm32f7_sai_s *priv = (struct stm32f7_sai_s *)arg;
  struct sai_buffer_s *bfcontainer;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* When the transfer was started, the active buffer containers were removed
   * from the pend queue and saved in the act queue.  We get here when the
   * transfer is finished... either successfully, with an error, or with a
   * timeout.
   *
   * In any case, the buffer containers in act will be moved to the end
   * of the done queue and act will be emptied before this worker is
   * started.
   */

  i2sinfo("act.head=%p done.head=%p\n", priv->act.head, priv->done.head);

  /* Check if IDLE */

  if (sq_empty(&priv->act))
    {
      /* Then start the next transfer.  This must be done with interrupts
       * disabled.
       */

      flags = enter_critical_section();
#ifdef CONFIG_STM32F7_SAI_DMA
      sai_dma_setup(priv);
#endif
      leave_critical_section(flags);
    }

  /* Process each buffer in the done queue */

  while (sq_peek(&priv->done) != NULL)
    {
      /* Remove the buffer container from the done queue.  NOTE that
       * interrupts must be enabled to do this because the done queue is
       * also modified from the interrupt level.
       */

      flags = enter_critical_section();
      bfcontainer = (struct sai_buffer_s *)sq_remfirst(&priv->done);
      leave_critical_section(flags);

      /* Perform the transfer done callback */

      DEBUGASSERT(bfcontainer && bfcontainer->callback);
      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, bfcontainer->result);

      /* Release our reference on the audio buffer.  This may very likely
       * cause the audio buffer to be freed.
       */

      apb_free(bfcontainer->apb);

      /* And release the buffer container */

      sai_buf_free(priv, bfcontainer);
    }
}

/****************************************************************************
 * Name: sai_schedule
 *
 * Description:
 *   An transfer completion or timeout has occurred.  Schedule processing on
 *   the working thread.
 *
 * Input Parameters:
 *   priv   - SAI state instance
 *   result - The result of the transfer
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Interrupts are disabled
 *   - The timeout has been canceled.
 *
 ****************************************************************************/

static void sai_schedule(struct stm32f7_sai_s *priv, int result)
{
  struct sai_buffer_s *bfcontainer;
  int ret;

  /* Move all entries from the act queue to the done queue */

  while (!sq_empty(&priv->act))
    {
      /* Remove the next buffer container from the act list */

      bfcontainer = (struct sai_buffer_s *)sq_remfirst(&priv->act);

      /* Report the result of the transfer */

      bfcontainer->result = result;

      /* Add the completed buffer container to the tail of the done queue */

      sq_addlast((sq_entry_t *)bfcontainer, &priv->done);
    }

  /* If the worker has completed running, then reschedule the working thread.
   * REVISIT:  There may be a race condition here.  So we do nothing is the
   * worker is not available.
   */

  if (work_available(&priv->work))
    {
      /* Schedule the done processing to occur on the worker thread. */

      ret = work_queue(HPWORK, &priv->work, sai_worker, priv, 0);
      if (ret != 0)
        {
          i2serr("ERROR: Failed to queue work: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: sai_dma_callback
 *
 * Description:
 *   This callback function is invoked at the completion of the SAI DMA.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   isr    - The interrupt status of the DMA transfer
 *   arg    - A pointer to the SAI state instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_SAI_DMA
static void sai_dma_callback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
  struct stm32f7_sai_s *priv = (struct stm32f7_sai_s *)arg;
  DEBUGASSERT(priv);

  /* Cancel the watchdog timeout */

  wd_cancel(&priv->dog);

  /* Then schedule completion of the transfer to occur on the worker thread */

  sai_schedule(priv, (isr & DMA_STREAM_TEIF_BIT) ? -EIO : OK);
}
#endif

/****************************************************************************
 * Name: sai_samplerate
 *
 * Description:
 *   Set the I2S RX/TX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static uint32_t sai_samplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  struct stm32f7_sai_s *priv = (struct stm32f7_sai_s *)dev;

  DEBUGASSERT(priv && rate > 0);

  /* Call callback to change system clock (needed for STM32F746 Disco) */

  if (priv->sampleratecb != NULL)
    {
      priv->frequency = priv->sampleratecb(dev, rate);
    }
  else
    {
      i2sinfo("No Sample Rate CB set!\n");
    }

  /* Save the new sample rate and update the divider */

  priv->samplerate = rate;
  sai_mckdivider(priv);

  return sai_getbitrate(priv);
}

/****************************************************************************
 * Name: sai_datawidth
 *
 * Description:
 *   Set the I2S data width.  The bitrate is determined by
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

static uint32_t sai_datawidth(struct i2s_dev_s *dev, int bits)
{
  struct stm32f7_sai_s *priv = (struct stm32f7_sai_s *)dev;
  uint32_t setbits;

  DEBUGASSERT(priv && bits >= 8);

  switch (bits)
    {
      case 8:
        setbits = SAI_CR1_DS_8BITS;
        break;

      case 16:
        setbits = SAI_CR1_DS_16BITS;
        break;

      case 32:
        setbits = SAI_CR1_DS_32BITS;
        break;

      default:
        i2serr("ERROR: Unsupported or invalid data width: %d\n", bits);
        return 0;
    }

  sai_modifyreg(priv, STM32F7_SAI_CR1_OFFSET, SAI_CR1_DS_MASK, setbits);

  sai_modifyreg(priv, STM32F7_SAI_FRCR_OFFSET,
                SAI_FRCR_FSALL_MASK | SAI_FRCR_FRL_MASK,
                SAI_FRCR_FSALL(bits) | SAI_FRCR_FRL(bits * 2));

  /* Save the new data width */

  priv->datalen = bits;

  return sai_getbitrate(priv);
}

/****************************************************************************
 * Name: sai_receive
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

static int sai_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                       i2s_callback_t callback, void *arg, uint32_t timeout)
{
  struct stm32f7_sai_s *priv = (struct stm32f7_sai_s *)dev;
  struct sai_buffer_s *bfcontainer;
  uint32_t mode;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv && apb);
  i2sinfo("apb=%p nbytes=%d arg=%p timeout=%" PRId32 "\n",
          apb, apb->nbytes - apb->curbyte, arg, timeout);

  /* Allocate a buffer container in advance */

  bfcontainer = sai_buf_allocate(priv);
  DEBUGASSERT(bfcontainer);

  /* Get exclusive access to the SAI driver data */

  sai_exclsem_take(priv);

  /* Verify not already TX'ing */

  if (priv->txenab)
    {
      i2serr("ERROR: SAI has no receiver\n");
      ret = -EAGAIN;
      goto errout_with_exclsem;
    }

  mode = priv->syncen ? SAI_CR1_MODE_SLAVE_RX : SAI_CR1_MODE_MASTER_RX;
  sai_modifyreg(priv, STM32F7_SAI_CR1_OFFSET, SAI_CR1_MODE_MASK, mode);
  priv->rxenab = true;

  /* Add a reference to the audio buffer */

  apb_reference(apb);

  /* Initialize the buffer container structure */

  bfcontainer->callback = (void *)callback;
  bfcontainer->timeout  = timeout;
  bfcontainer->arg      = arg;
  bfcontainer->apb      = apb;
  bfcontainer->result   = -EBUSY;

  /* Add the buffer container to the end of the pending queue */

  flags = enter_critical_section();
  sq_addlast((sq_entry_t *)bfcontainer, &priv->pend);

  /* Then start the next transfer.  If there is already a transfer in
   * progress, then this will do nothing.
   */

#ifdef CONFIG_STM32F7_SAI_DMA
  ret = sai_dma_setup(priv);
#endif
  DEBUGASSERT(ret == OK);
  leave_critical_section(flags);
  sai_exclsem_give(priv);
  return OK;

errout_with_exclsem:
  sai_exclsem_give(priv);
  sai_buf_free(priv, bfcontainer);
  return ret;
}

/****************************************************************************
 * Name: sai_send
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

static int sai_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                    i2s_callback_t callback, void *arg, uint32_t timeout)
{
  struct stm32f7_sai_s *priv = (struct stm32f7_sai_s *)dev;
  struct sai_buffer_s *bfcontainer;
  uint32_t mode;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv && apb);
  i2sinfo("apb=%p nbytes=%d arg=%p timeout=%" PRId32 "\n",
          apb, apb->nbytes - apb->curbyte, arg, timeout);

  /* Allocate a buffer container in advance */

  bfcontainer = sai_buf_allocate(priv);
  DEBUGASSERT(bfcontainer);

  /* Get exclusive access to the SAI driver data */

  sai_exclsem_take(priv);

  /* Verify not already RX'ing */

  if (priv->rxenab)
    {
      i2serr("ERROR: SAI has no transmitter\n");
      ret = -EAGAIN;
      goto errout_with_exclsem;
    }

  mode = priv->syncen ? SAI_CR1_MODE_SLAVE_TX : SAI_CR1_MODE_MASTER_TX;
  sai_modifyreg(priv, STM32F7_SAI_CR1_OFFSET, SAI_CR1_MODE_MASK, mode);
  priv->txenab = true;

  /* Add a reference to the audio buffer */

  apb_reference(apb);

  /* Initialize the buffer container structure */

  bfcontainer->callback = (void *)callback;
  bfcontainer->timeout  = timeout;
  bfcontainer->arg      = arg;
  bfcontainer->apb      = apb;
  bfcontainer->result   = -EBUSY;

  /* Add the buffer container to the end of the pending queue */

  flags = enter_critical_section();
  sq_addlast((sq_entry_t *)bfcontainer, &priv->pend);

  /* Then start the next transfer.  If there is already a transfer in
   * progress, then this will do nothing.
   */

#ifdef CONFIG_STM32F7_SAI_DMA
  ret = sai_dma_setup(priv);
#endif
  DEBUGASSERT(ret == OK);
  leave_critical_section(flags);
  sai_exclsem_give(priv);
  return OK;

errout_with_exclsem:
  sai_exclsem_give(priv);
  sai_buf_free(priv, bfcontainer);
  return ret;
}

/****************************************************************************
 * Name: sai_bufsem_take
 *
 * Description:
 *   Take the buffer semaphore handling any exceptional conditions
 *
 * Input Parameters:
 *   priv - A reference to the SAI peripheral state
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void sai_bufsem_take(struct stm32f7_sai_s *priv)
{
  int ret;

  /* Wait until we successfully get the semaphore.  EINTR is the only
   * expected 'failure' (meaning that the wait for the semaphore was
   * interrupted by a signal).
   */

  do
    {
      ret = nxsem_wait(&priv->bufsem);
      DEBUGASSERT(ret == 0 || ret == -EINTR);
    }
  while (ret == -EINTR);
}

/****************************************************************************
 * Name: sai_buf_allocate
 *
 * Description:
 *   Allocate a buffer container by removing the one at the head of the
 *   free list
 *
 * Input Parameters:
 *   priv - SAI state instance
 *
 * Returned Value:
 *   A non-NULL pointer to the allocate buffer container on success; NULL if
 *   there are no available buffer containers.
 *
 * Assumptions:
 *   The caller does NOT have exclusive access to the SAI state structure.
 *   That would result in a deadlock!
 *
 ****************************************************************************/

static struct sai_buffer_s *sai_buf_allocate(struct stm32f7_sai_s *priv)
{
  struct sai_buffer_s *bfcontainer;
  irqstate_t flags;

  /* Set aside a buffer container.  By doing this, we guarantee that we will
   * have at least one free buffer container.
   */

  sai_bufsem_take(priv);

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
 * Name: sai_buf_free
 *
 * Description:
 *   Free buffer container by adding it to the head of the free list
 *
 * Input Parameters:
 *   priv - SAI state instance
 *   bfcontainer - The buffer container to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller has exclusive access to the SAI state structure
 *
 ****************************************************************************/

static void sai_buf_free(struct stm32f7_sai_s *priv,
                         struct sai_buffer_s *bfcontainer)
{
  irqstate_t flags;

  /* Put the buffer container back on the free list */

  flags = enter_critical_section();
  bfcontainer->flink  = priv->freelist;
  priv->freelist = bfcontainer;
  leave_critical_section(flags);

  /* Wake up any threads waiting for a buffer container */

  sai_bufsem_give(priv);
}

/****************************************************************************
 * Name: sai_buf_initialize
 *
 * Description:
 *   Initialize the buffer container allocator by adding all of the
 *   pre-allocated buffer containers to the free list
 *
 * Input Parameters:
 *   priv - SAI state instance
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in SAI initialization so that there are no issues with
 *   concurrency.
 *
 ****************************************************************************/

static void sai_buf_initialize(struct stm32f7_sai_s *priv)
{
  int i;

  priv->freelist = NULL;
  nxsem_init(&priv->bufsem, 0, CONFIG_STM32F7_SAI_MAXINFLIGHT);

  for (i = 0; i < CONFIG_STM32F7_SAI_MAXINFLIGHT; i++)
    {
      sai_buf_free(priv, &priv->containers[i]);
    }
}

/****************************************************************************
 * Name: sai_portinitialize
 *
 * Description:
 *   Initialize the selected SAI port in its default state
 *
 * Input Parameters:
 *   priv   - private SAI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_portinitialize(struct stm32f7_sai_s *priv)
{
  sai_dump_regs(priv, "Before initialization");

  nxsem_init(&priv->exclsem, 0, 1);

  /* Initialize buffering */

  sai_buf_initialize(priv);

  /* Configure the master clock divider */

  sai_mckdivider(priv);

  /* Configure the data width */

  sai_datawidth((struct i2s_dev_s *)priv,
                CONFIG_STM32F7_SAI_DEFAULT_DATALEN);

#ifdef CONFIG_STM32F7_SAI_DMA
  /* Get DMA channel */

  priv->dma = stm32_dmachannel(priv->dma_ch);
  DEBUGASSERT(priv->dma);

  sai_modifyreg(priv, STM32F7_SAI_CR1_OFFSET, 0, SAI_CR1_DMAEN);
#endif

  sai_modifyreg(priv, STM32F7_SAI_CR1_OFFSET, SAI_CR1_SYNCEN_MASK,
                priv->syncen);

  sai_modifyreg(priv, STM32F7_SAI_CR1_OFFSET, 0, SAI_CR1_OUTDRIV);

  sai_modifyreg(priv, STM32F7_SAI_CR2_OFFSET, SAI_CR2_FTH_MASK,
                SAI_CR2_FTH_1QF);

  sai_modifyreg(priv, STM32F7_SAI_FRCR_OFFSET,
                SAI_FRCR_FSDEF | SAI_FRCR_FSPOL | SAI_FRCR_FSOFF,
                SAI_FRCR_FSDEF_CHID | SAI_FRCR_FSPOL_LOW |
                  SAI_FRCR_FSOFF_BFB);

  sai_modifyreg(priv, STM32F7_SAI_SLOTR_OFFSET,
                SAI_SLOTR_NBSLOT_MASK | SAI_SLOTR_SLOTEN_MASK,
                SAI_SLOTR_NBSLOT(2) | SAI_SLOTR_SLOTEN_0 |
                  SAI_SLOTR_SLOTEN_1);

  sai_modifyreg(priv, STM32F7_SAI_CR1_OFFSET, 0, SAI_CR1_SAIEN);
  sai_dump_regs(priv, "After initialization");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_sai_initialize
 *
 * Description:
 *   Initialize the selected SAI block
 *
 * Input Parameters:
 *   intf - I2S interface number (identifying the "logical" SAI interface)
 *
 * Returned Value:
 *   Valid I2S device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2s_dev_s *stm32_sai_initialize(int intf,
                                       stm32_sai_sampleratecb_t sampleratecb)
{
  struct stm32f7_sai_s *priv;
  irqstate_t flags;

  flags = enter_critical_section();

  switch (intf)
    {
#ifdef CONFIG_STM32F7_SAI1_A
      case SAI1_BLOCK_A:
        {
          i2sinfo("SAI1 Block A Selected\n");
          priv = &g_sai1a_priv;
          priv->sampleratecb = sampleratecb;

          stm32_configgpio(GPIO_SAI1_SD_A);
#  ifndef CONFIG_STM32F7_SAI1_A_SYNC_WITH_B
          stm32_configgpio(GPIO_SAI1_FS_A);
          stm32_configgpio(GPIO_SAI1_SCK_A);
          stm32_configgpio(GPIO_SAI1_MCLK_A);
#  endif
          break;
        }
#endif

#ifdef CONFIG_STM32F7_SAI1_B
      case SAI1_BLOCK_B:
        {
          i2sinfo("SAI1 Block B Selected\n");
          priv = &g_sai1b_priv;
          priv->sampleratecb = sampleratecb;

          stm32_configgpio(GPIO_SAI1_SD_B);
#  ifndef CONFIG_STM32F7_SAI1_B_SYNC_WITH_A
          stm32_configgpio(GPIO_SAI1_FS_B);
          stm32_configgpio(GPIO_SAI1_SCK_B);
          stm32_configgpio(GPIO_SAI1_MCLK_B);
#  endif
          break;
        }
#endif

#ifdef CONFIG_STM32F7_SAI2_A
      case SAI2_BLOCK_A:
        {
          i2sinfo("SAI2 Block A Selected\n");
          priv = &g_sai2a_priv;
          priv->sampleratecb = sampleratecb;

          stm32_configgpio(GPIO_SAI2_SD_A);
#  ifndef CONFIG_STM32F7_SAI2_A_SYNC_WITH_B
          stm32_configgpio(GPIO_SAI2_FS_A);
          stm32_configgpio(GPIO_SAI2_SCK_A);
          stm32_configgpio(GPIO_SAI2_MCLK_A);
#  endif
          break;
        }
#endif

#ifdef CONFIG_STM32F7_SAI2_B
      case SAI2_BLOCK_B:
        {
          i2sinfo("SAI2 Block B Selected\n");
          priv = &g_sai2b_priv;
          priv->sampleratecb = sampleratecb;

          stm32_configgpio(GPIO_SAI2_SD_B);
#  ifndef CONFIG_STM32F7_SAI2_B_SYNC_WITH_A
          stm32_configgpio(GPIO_SAI2_FS_B);
          stm32_configgpio(GPIO_SAI2_SCK_B);
          stm32_configgpio(GPIO_SAI2_MCLK_B);
#  endif
          break;
        }
#endif

      default:
        {
          i2sinfo("No SAI interface defined\n");
          goto err;
        }
    }

  sai_portinitialize(priv);
  leave_critical_section(flags);

  return &priv->dev;

err:
  leave_critical_section(flags);
  return NULL;
}

#endif

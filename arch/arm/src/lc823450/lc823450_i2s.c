/****************************************************************************
 * arch/arm/src/lc823450/lc823450_i2s.c
 *
 *   Copyright 2017,2018 Sony Video & Sound Products Inc.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <semaphore.h>
#include <nuttx/arch.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>

#ifdef CONFIG_SMP
#  include <nuttx/signal.h>
#endif

#include "up_arch.h"
#include "lc823450_dma.h"
#include "lc823450_i2s.h"
#include "lc823450_syscontrol.h"
#include "lc823450_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LC823450_AUDIO_REGBASE  0x40060000

#define ABUF_REGBASE     (LC823450_AUDIO_REGBASE + 0x0000)
#define BEEP_REGBASE     (LC823450_AUDIO_REGBASE + 0x1200)
#define PCKGEN_REGBASE   (LC823450_AUDIO_REGBASE + 0x1600)
#define AUDCTL_REGBASE   (LC823450_AUDIO_REGBASE + 0x4000)

#define ABUFCLR    (ABUF_REGBASE + 0x0000)

#define ABUFACCEN  (ABUF_REGBASE + 0x0004)
#define   ABUFACCEN_CDCFEN  (1 << 5)

#define ABUFIRQEN0   (ABUF_REGBASE + 0x0008)
#define   ABUFIRQEN0_BFULIRQEN (1 << 5)

#define ABUFSTS1     (ABUF_REGBASE + 0x0034)

#define BUF_F_BASE   (ABUF_REGBASE + 0x00c0 + (0x4 * 5))
#define BUF_F_SIZE   (ABUF_REGBASE + 0x0100 + (0x4 * 5))
#define BUF_F_ULVL   (ABUF_REGBASE + 0x0140 + (0x4 * 5))
#define BUF_F_DTCAP  (ABUF_REGBASE + 0x01c0 + (0x4 * 5))
#define BUF_F_ACCESS (ABUF_REGBASE + 0x0300 + (0x4 * 5))

#define CLOCKEN    (AUDCTL_REGBASE + 0x0000)
#define   CLOCKEN_FCE_PCKGEN   (1 << 28)
#define   CLOCKEN_FCE_PCMPS0   (1 << 17)
#define   CLOCKEN_FCE_BEEP     (1 << 16)
#define   CLOCKEN_FCE_VOLPS0   (1 << 13)

#define AUDSEL     (AUDCTL_REGBASE + 0x001c)
#define   AUDSEL_PCM0_MODE     (1 << 17)
#define   AUDSEL_PCM0_MODEM    (1 << 16)

#define PSCTL      (AUDCTL_REGBASE + 0x0110)

#define PCMOUTEN   (AUDCTL_REGBASE + 0x0500)
#define   PCMOUTEN_DOUT0EN     (1 <<  3)
#define   PCMOUTEN_LRCK0EN     (1 <<  2)
#define   PCMOUTEN_MCLK0EN     (1 <<  1)
#define   PCMOUTEN_BCK0EN      (1 <<  0)

#define PCMCTL     (AUDCTL_REGBASE + 0x0504)

#define BEEP_CTL     (BEEP_REGBASE + 0x0000)
#define BEEP_BYPASS  (BEEP_REGBASE + 0x0004)
#define BEEP_COEFF   (BEEP_REGBASE + 0x0008)
#define BEEP_TIME    (BEEP_REGBASE + 0x000c)

/* Audio PLL */

#define AUDIOPLL_REGBASE (LC823450_OSCSYS_REGBASE + 0x2000)
#define AUDPLLCNT        (AUDIOPLL_REGBASE + 0x00)
#define AUDPLLMDIV       (AUDIOPLL_REGBASE + 0x04)
#define AUDPLLNDIV       (AUDIOPLL_REGBASE + 0x08)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The state of the one I2S peripheral */

struct lc823450_i2s_s
{
  struct i2s_dev_s  dev;        /* Externally visible I2S interface */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint32_t lc823450_i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t lc823450_i2s_txdatawidth(struct i2s_dev_s *dev, int bits);
static int      lc823450_i2s_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                                  i2s_callback_t callback, void *arg,
                                  uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2S device operations */

static const struct i2s_ops_s g_i2sops =
{
  /* Transmitter methods */

  .i2s_txsamplerate = lc823450_i2s_txsamplerate,
  .i2s_txdatawidth  = lc823450_i2s_txdatawidth,
  .i2s_send         = lc823450_i2s_send,
};

static DMA_HANDLE _htxdma;
static sem_t      _sem_txdma;
static sem_t      _sem_buf_under;

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern unsigned int XT1OSC_CLK;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _setup_audio_pll
 ****************************************************************************/

static void _setup_audio_pll(uint32_t freq)
{
  ASSERT(24000000 == XT1OSC_CLK);

  uint32_t m;
  uint32_t n;

  switch (freq)
    {
      case 44100:
        m = 625;
        n = 3528;
        break;

      case 48000:
        m = 125;
        n = 768;
        break;

      default:
        ASSERT(false);
    }

  /* Set divider */

  putreg32(n, AUDPLLNDIV);
  putreg32(m, AUDPLLMDIV);

  /* Audio PLL standby=off, Audio PLL unreset */

  putreg32(0x0503, AUDPLLCNT);

  /* TODO: Wait */

  usleep(50 * 1000);

  /* Switch to the PLL */

  modifyreg32(AUDCLKCNT,
              0x0,
              0x03 /* AUDCLKSEL=Audio PLL */
              );

  /* TODO: Clock divider settings */

  modifyreg32(AUDCLKCNT,
              0x0,
              0x0200  /* AUDDIV=2 */
              );
}

/****************************************************************************
 * Name: _i2s_txdma_callback
 ****************************************************************************/

static void _i2s_txdma_callback(DMA_HANDLE hdma, void *arg, int result)
{
  sem_t *waitsem = (sem_t *)arg;
  nxsem_post(waitsem);
}

/****************************************************************************
 * Name: _i2s_semtake
 ****************************************************************************/

static void _i2s_semtake(FAR sem_t *sem)
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
 * Name: lc823450_i2s_txsamplerate
 ****************************************************************************/

static uint32_t lc823450_i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  /* TODO */
  return 0;
}

/****************************************************************************
 * Name: lc823450_i2s_txdatawidth
 ****************************************************************************/

static uint32_t lc823450_i2s_txdatawidth(struct i2s_dev_s *dev, int bits)
{
  /* TODO */
  return 0;
}

/****************************************************************************
 * Name: _i2s_isr
 ****************************************************************************/

static int _i2s_isr(int irq, FAR void *context, FAR void *arg)
{
  /* Disable Buffer F Under Level IRQ */

  putreg32(0, ABUFIRQEN0);

  /* post semaphore for the waiter */

  nxsem_post(&_sem_buf_under);
  return 0;
}

/****************************************************************************
 * Name: lc823450_i2s_send
 ****************************************************************************/

static int lc823450_i2s_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                             i2s_callback_t callback, void *arg,
                             uint32_t timeout)
{
  /* Enable Buffer F Under Level IRQ */

  putreg32(ABUFIRQEN0_BFULIRQEN, ABUFIRQEN0);

  /* Wait for Audio Buffer */

  _i2s_semtake(&_sem_buf_under);

  volatile uint32_t *ptr = (uint32_t *)&apb->samp[apb->curbyte];
  uint32_t n = apb->nbytes;

  /* Setup and start DMA for I2S */

  lc823450_dmasetup(_htxdma,
                    LC823450_DMA_SRCINC |
                    LC823450_DMA_SRCWIDTH_WORD |
                    LC823450_DMA_DSTWIDTH_WORD,
                    (uint32_t)ptr, (uint32_t)BUF_F_ACCESS, n / 4);

  lc823450_dmastart(_htxdma,
                    _i2s_txdma_callback,
                    &_sem_txdma);

  _i2s_semtake(&_sem_txdma);

  /* Invoke the callback handler */

  callback(dev, apb, arg, 0);
  return OK;
}

/****************************************************************************
 * Name: lc823450_i2s_beeptest
 ****************************************************************************/

#ifdef BEEP_TEST
static void lc823450_i2s_beeptest(void)
{
  /* Set BEEP params */

  putreg32(0x0,    BEEP_BYPASS);
  putreg32(0x123ca6, BEEP_COEFF); /* 1kHz@fs=44.1k */
  putreg32(0xffff, BEEP_TIME);

  /* Start */

  putreg32(0x3,    BEEP_CTL);
}
#endif

/****************************************************************************
 * Name: lc823450_i2s_configure
 ****************************************************************************/

static int lc823450_i2s_configure(void)
{
  _setup_audio_pll(44100);

  /* Unreset Audio Buffer */

  putreg32(MRSTCNTEXT3_AUDIOBUF_RSTB,
           MRSTCNTEXT3);

  /* Enable clock to Audio Buffer */

  putreg32(MCLKCNTEXT3_AUDIOBUF_CLKEN,
           MCLKCNTEXT3);

  /* F Buffer = 32KB */

  putreg32(4096 * 8, BUF_F_SIZE);

  /* Buffer Under Level = 1KB */

  putreg32(1024, BUF_F_ULVL);

  /* Clear Audio Buffer */

  putreg32(0xffff, ABUFCLR);

  /* Access Enable */

  putreg32(ABUFACCEN_CDCFEN, ABUFACCEN);

  /* PCM0: BCK0/LRCK0=master, MCLK0=master */

  putreg32(AUDSEL_PCM0_MODE |
           AUDSEL_PCM0_MODEM,
           AUDSEL);

  /* LRCK0/BCK0: 1/1fs, BCK0:64fs, BCK1:64fs */

  putreg32(0x00001010,
           PCMCTL);

  /* Enable DOUT0/LRCK0/MCL0/BCK0 */

  putreg32(PCMOUTEN_DOUT0EN |
           PCMOUTEN_LRCK0EN |
           PCMOUTEN_MCLK0EN |
           PCMOUTEN_BCK0EN,
           PCMOUTEN);

  /* Stereo, PCMDLY=1, LRCK active low,
   * MSB first and left justified, 32bit
   */

  putreg32(0x64, PSCTL);

  /* Enable PCMPS0 */

  putreg32(CLOCKEN_FCE_PCKGEN |
           CLOCKEN_FCE_BEEP |
           CLOCKEN_FCE_PCMPS0 |
           CLOCKEN_FCE_VOLPS0,
           CLOCKEN);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_i2sdev_initialize
 ****************************************************************************/

FAR struct i2s_dev_s *lc823450_i2sdev_initialize(void)
{
  FAR struct lc823450_i2s_s *priv = NULL;

  /* The support STM32 parts have only a single I2S port */

  i2sinfo("port: %d\n", port);

  /* Allocate a new state structure for this chip select.  NOTE that there
   * is no protection if the same chip select is used in two different
   * chip select structures.
   */

  priv = (struct lc823450_i2s_s *)zalloc(sizeof(struct lc823450_i2s_s));
  if (!priv)
    {
      i2serr("ERROR: Failed to allocate a chip select structure\n");
      return NULL;
    }

  /* Initialize the common parts for the I2S device structure */

  priv->dev.ops = &g_i2sops;

  (void)lc823450_i2s_configure();

#ifdef BEEP_TEST
  lc823450_i2s_beeptest();
#endif

  _htxdma = lc823450_dmachannel(DMA_CHANNEL_VIRTUAL);
  nxsem_init(&_sem_txdma, 0, 0);
  nxsem_init(&_sem_buf_under, 0, 0);

#ifdef CONFIG_SMP
  cpu_set_t cpuset0;
  cpu_set_t cpuset1;

  CPU_ZERO(&cpuset1);
  CPU_SET(0, &cpuset1);

  /* Backup the current affinity */

  (void)nxsched_getaffinity(getpid(), sizeof(cpuset0), &cpuset0);

  /* Set the new affinity which assigns to CPU0 */

  (void)nxsched_setaffinity(getpid(), sizeof(cpuset1), &cpuset1);
  nxsig_usleep(10 * 1000);
#endif

  irq_attach(LC823450_IRQ_AUDIOBUF0, _i2s_isr, NULL);

  /* Enable IRQ for Audio Buffer */

  up_enable_irq(LC823450_IRQ_AUDIOBUF0);

#ifdef CONFIG_SMP
  /* Restore the original affinity */

  (void)nxsched_setaffinity(getpid(), sizeof(cpuset0), &cpuset0);
  nxsig_usleep(10 * 1000);
#endif

  /* Success exit */

  return &priv->dev;
}

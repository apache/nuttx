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

/* #define SHOW_BUFFERING */

#define I2S_HAS_IOCTL

#define BUFID(n) (n - 'A')

#define LC823450_AUDIO_REGBASE  0x40060000

#define ABUF_REGBASE     (LC823450_AUDIO_REGBASE + 0x0000)
#define SSRC_REGBASE     (LC823450_AUDIO_REGBASE + 0x1000)
#define BEEP_REGBASE     (LC823450_AUDIO_REGBASE + 0x1200)
#define DGMIC_REGBASE    (LC823450_AUDIO_REGBASE + 0x1500)
#define PCKGEN_REGBASE   (LC823450_AUDIO_REGBASE + 0x1600)
#define ASRC_REGBASE     (LC823450_AUDIO_REGBASE + 0x1a00)
#define MP3DEC_REGBASE   (LC823450_AUDIO_REGBASE + 0x2000)
#define AUDCTL_REGBASE   (LC823450_AUDIO_REGBASE + 0x4000)

/* Audio Buffer */

#define ABUFCLR    (ABUF_REGBASE + 0x0000)

#define ABUFACCEN  (ABUF_REGBASE + 0x0004)
#define   ABUFACCEN_RDCTEN(n) (1 << (BUFID(n) + 16))
#define   ABUFACCEN_CDCEN(n)  (1 << (BUFID(n) + 0))

#define ABUFIRQEN0   (ABUF_REGBASE + 0x0008)
#define   ABUFIRQEN0_BOLIRQEN(n) (1 << (BUFID(n) + 16))
#define   ABUFIRQEN0_BULIRQEN(n) (1 << (BUFID(n) + 0))

#define ABUFSTS1     (ABUF_REGBASE + 0x0034)
#define   ABUFSTS1_BOLVL(n)  (1 << (BUFID(n) + 16))
#define   ABUFSTS1_BULVL(n)  (1 << (BUFID(n) + 0))

#define BUF_BASE(n)   (ABUF_REGBASE + 0x00c0 + (4 * BUFID(n)))
#define BUF_SIZE(n)   (ABUF_REGBASE + 0x0100 + (4 * BUFID(n)))
#define BUF_ULVL(n)   (ABUF_REGBASE + 0x0140 + (4 * BUFID(n)))
#define BUF_OLVL(n)   (ABUF_REGBASE + 0x0180 + (4 * BUFID(n)))
#define BUF_DTCAP(n)  (ABUF_REGBASE + 0x01c0 + (4 * BUFID(n)))
#define BUF_ACCESS(n) (ABUF_REGBASE + 0x0300 + (4 * BUFID(n)))

#define BUFCTL(n)     (ABUF_REGBASE + 0x0080 + (4 * BUFID(n)))
#define   BUFCTL_MONO  (1 << 0)

/* SSRC */

#define SSRC_MODE     (SSRC_REGBASE + 0x0000)
#define SSRC_FSI      (SSRC_REGBASE + 0x0004)
#define SSRC_FSO      (SSRC_REGBASE + 0x0008)
#define SSRC_SRESETB  (SSRC_REGBASE + 0x0010)
#define SSRC_STATUS   (SSRC_REGBASE + 0x0014)

/* Audio Control */

#define CLOCKEN    (AUDCTL_REGBASE + 0x000)
#define   CLOCKEN_FCE_ASRC     (1 << 29)
#define   CLOCKEN_FCE_PCKGEN   (1 << 28)
#define   CLOCKEN_FCE_PCMPS0   (1 << 17)
#define   CLOCKEN_FCE_BEEP     (1 << 16)
#define   CLOCKEN_FCE_VOLPS0   (1 << 13)
#define   CLOCKEN_FCE_VOLSP0   (1 << 11)
#define   CLOCKEN_FCE_DGMIC    (1 << 7)
#define   CLOCKEN_FCE_VOLD     (1 << 6)
#define   CLOCKEN_FCE_SSRC     (1 << 4)
#define   CLOCKEN_FCE_MUTED    (1 << 3)
#define   CLOCKEN_FCE_MP3DEC   (1 << 2)

#define AUDSEL     (AUDCTL_REGBASE + 0x01c)
#define   AUDSEL_PCM0_MODE     (1 << 17)
#define   AUDSEL_PCM0_MODEM    (1 << 16)
#define   AUDSEL_PCMSEL        (1 << 4)
#define   AUDSEL_DECSEL        (1 << 0)

#define PSCTL      (AUDCTL_REGBASE + 0x110)

/* Volume (SP0) */

#define VOLSP0_CONT (AUDCTL_REGBASE + 0x320)
#define   VOL_CONT_DIRECT      (1 << 20)

#define PCMOUTEN   (AUDCTL_REGBASE + 0x500)
#define   PCMOUTEN_DOUT0EN     (1 <<  3)
#define   PCMOUTEN_LRCK0EN     (1 <<  2)
#define   PCMOUTEN_MCLK0EN     (1 <<  1)
#define   PCMOUTEN_BCK0EN      (1 <<  0)

#define PCMCTL     (AUDCTL_REGBASE + 0x504)

/* BEEP */

#define BEEP_CTL     (BEEP_REGBASE + 0x0000)
#define BEEP_BYPASS  (BEEP_REGBASE + 0x0004)
#define BEEP_COEFF   (BEEP_REGBASE + 0x0008)
#define BEEP_TIME    (BEEP_REGBASE + 0x000c)

/* Audio PLL */

#define AUDIOPLL_REGBASE (LC823450_OSCSYS_REGBASE + 0x2000)
#define AUDPLLCNT        (AUDIOPLL_REGBASE + 0x00)
#define AUDPLLMDIV       (AUDIOPLL_REGBASE + 0x04)
#define AUDPLLNDIV       (AUDIOPLL_REGBASE + 0x08)

#define DGMICCTL         (DGMIC_REGBASE + 0x00)
#define ALCCTL           (DGMIC_REGBASE + 0x30)

/* ASRC */

#define ASRC_MODE        (ASRC_REGBASE + 0x0000)
#define ASRC_FSI         (ASRC_REGBASE + 0x0004)
#define ASRC_FSO         (ASRC_REGBASE + 0x0008)
#define ASRC_SRESETB     (ASRC_REGBASE + 0x0010)
#define ASRC_STATUS      (ASRC_REGBASE + 0x0014)

/* MP3 Decoder */

#define MP3DEC_ERR       (MP3DEC_REGBASE + 0x08)
#define MP3DEC_BUFM      (MP3DEC_REGBASE + 0x1c)
#define MP3DEC_ERRMODE   (MP3DEC_REGBASE + 0x20)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The state of the one I2S peripheral */

struct lc823450_i2s_s
{
  struct i2s_dev_s  dev;        /* Externally visible I2S interface */
};

static bool     _b_input_started = false;
static uint32_t _i2s_tx_th_bytes;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint32_t lc823450_i2s_rxsamplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t lc823450_i2s_rxdatawidth(struct i2s_dev_s *dev, int bits);
static int      lc823450_i2s_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                                     i2s_callback_t callback, void *arg,
                                     uint32_t timeout);

static uint32_t lc823450_i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t lc823450_i2s_txdatawidth(struct i2s_dev_s *dev, int bits);
static int      lc823450_i2s_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                                  i2s_callback_t callback, void *arg,
                                  uint32_t timeout);

static void     lc823450_i2s_setchannel(char id, uint8_t ch);
static void     lc823450_i2s_mp3dec(bool enable);

static int      lc823450_i2s_ioctl(struct i2s_dev_s *dev,
                                   int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2S device operations */

static const struct i2s_ops_s g_i2sops =
{
  /* Receiver methods */

  .i2s_rxsamplerate = lc823450_i2s_rxsamplerate,
  .i2s_rxdatawidth  = lc823450_i2s_rxdatawidth,
  .i2s_receive      = lc823450_i2s_receive,

  /* Transmitter methods */

  .i2s_txsamplerate = lc823450_i2s_txsamplerate,
  .i2s_txdatawidth  = lc823450_i2s_txdatawidth,
  .i2s_send         = lc823450_i2s_send,

#ifdef I2S_HAS_IOCTL
  /* Ioctl */

  .i2s_ioctl        = lc823450_i2s_ioctl,
#endif
};

static DMA_HANDLE _hrxdma;
static sem_t      _sem_rxdma;
static sem_t      _sem_buf_over;

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
  DEBUGASSERT(24000000 == XT1OSC_CLK);

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
        DEBUGASSERT(false);
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
 * Name: lc823450_i2s_rxsamplerate
 ****************************************************************************/

static uint32_t lc823450_i2s_rxsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  /* Change ASRC FSO rate */

  /* Stop/Reset/Unreset ASRC */

  putreg32(0, ASRC_MODE);
  putreg32(0, ASRC_SRESETB);
  putreg32(1, ASRC_SRESETB);
  while (getreg32(ASRC_STATUS) & 0x1);

  /* Setup FSO */

  putreg32((rate) << 0, ASRC_FSO);

  /* Restart ASRC */

  putreg32(0x1, ASRC_MODE);
  while (getreg32(ASRC_STATUS) != 0x1);

  return 0;
}

/****************************************************************************
 * Name: lc823450_i2s_rxdatawidth
 ****************************************************************************/

static uint32_t lc823450_i2s_rxdatawidth(struct i2s_dev_s *dev, int bits)
{
  return 0;
}

/****************************************************************************
 * Name: lc823450_i2s_setchannel
 ****************************************************************************/

static void lc823450_i2s_setchannel(char id, uint8_t ch)
{
  switch (ch)
    {
      case 1:
        modifyreg32(BUFCTL(id), 0, BUFCTL_MONO);
        break;

      case 2:
        modifyreg32(BUFCTL(id), BUFCTL_MONO, 0);
        break;

      default:
        DEBUGASSERT(false);
        break;
    }

  modifyreg32(ABUFCLR, 0, BUFID(id));
}

/****************************************************************************
 * Name: _setup_tx_threshold (threshold to start playback)
 ****************************************************************************/

static void _setup_tx_threshold(uint32_t tx_th)
{
  if (0 == tx_th)
    {
      /* default tx threshould : 1024bytes */

      _i2s_tx_th_bytes = 1024;
    }
  else
    {
      /* tx_th: 0 to 100 (%) */

      _i2s_tx_th_bytes = getreg32(BUF_SIZE('C')) * tx_th / 100;
    }

  /* NOTE: Buffer Under Level is not controlled by tx threshold */
}

/****************************************************************************
 * Name: lc823450_i2s_rxdatawidth
 ****************************************************************************/

static int lc823450_i2s_ioctl(struct i2s_dev_s *dev, int cmd, unsigned long arg)
{
  FAR const struct audio_caps_desc_s *cap_desc;
  uint32_t tx_th;
  uint32_t rate[2];
  uint8_t  ch[2];
  uint8_t  fmt[2];

  switch (cmd)
    {
      case AUDIOIOC_CONFIGURE:
        cap_desc = (FAR const struct audio_caps_desc_s *)((uintptr_t)arg);
        DEBUGASSERT(NULL != cap_desc);

        tx_th   = cap_desc->caps.ac_controls.w >> 24;
        rate[1] = cap_desc->caps.ac_controls.w & 0xfffff;
        ch[1]   = cap_desc->caps.ac_channels;
        fmt[1]  = cap_desc->caps.ac_format.hw;

        if (cap_desc->caps.ac_type & AUDIO_TYPE_OUTPUT)
          {
            _setup_tx_threshold(tx_th);

            rate[0] = getreg32(SSRC_FSI) >> 13;
            ch[0]   = (getreg32(BUFCTL('C')) & BUFCTL_MONO) ? 1 : 2;
            fmt[0]  = getreg32(AUDSEL) & AUDSEL_DECSEL ? AUDIO_FMT_MP3 : AUDIO_FMT_PCM;

            if (rate[0] != rate[1])
              {
                audinfo("change output rate: %d -> %d \n", rate[0], rate[1]);
                lc823450_i2s_txsamplerate(dev, rate[1]);
              }

            if (ch[0] != ch[1])
              {
                audinfo("change output ch: %d -> %d \n", ch[0], ch[1]);
                lc823450_i2s_setchannel('C', ch[1]);
              }

            if (fmt[0] != fmt[1])
              {
                lc823450_i2s_mp3dec(fmt[1] == AUDIO_FMT_MP3 ? true : false);
              }
          }

        if (cap_desc->caps.ac_type & AUDIO_TYPE_INPUT)
          {
            rate[0] = getreg32(ASRC_FSO);
            ch[0]   = (getreg32(BUFCTL('J')) & BUFCTL_MONO) ? 1 : 2;

            if (rate[0] != rate[1])
              {
                audinfo("change input rate: %d -> %d \n", rate[0], rate[1]);
                lc823450_i2s_rxsamplerate(dev, rate[1]);
              }

            if (ch[0] != ch[1])
              {
                audinfo("change input ch: %d -> %d \n", ch[0], ch[1]);
                lc823450_i2s_setchannel('J', ch[1]);
              }
          }

        break;

      default:
        break;
    }

  return 0;
}

/****************************************************************************
 * Name: _i2s_rxdma_callback
 ****************************************************************************/

static void _i2s_rxdma_callback(DMA_HANDLE hdma, void *arg, int result)
{
  sem_t *waitsem = (sem_t *)arg;
  nxsem_post(waitsem);
}

/****************************************************************************
 * Name: lc823450_i2s_receive
 ****************************************************************************/

static int lc823450_i2s_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                                i2s_callback_t callback, void *arg,
                                uint32_t timeout)
{
#if 1 /* TODO: should move to rxsamplerate later */
  if (false == _b_input_started)
    {
      _b_input_started = true;

      /* Start J Buffer */

      modifyreg32(ABUFACCEN, 0, ABUFACCEN_CDCEN('J'));


      /* J Buffer : ACLTALN=0, ACLTEN=0 */

      modifyreg32(BUFCTL('J'), 0x3 << 8, 0);
    }
#endif

  /* Enable J Buffer Over Level IRQ */

  modifyreg32(ABUFIRQEN0, 0, ABUFIRQEN0_BOLIRQEN('J'));

  /* Wait for Audio Buffer */

  _i2s_semtake(&_sem_buf_over);

  volatile uint32_t *ptr = (uint32_t *)&apb->samp[apb->curbyte];
  uint32_t n = apb->nmaxbytes;

  /* Setup and start DMA for I2S */

  lc823450_dmasetup(_hrxdma,
                    LC823450_DMA_DSTINC |
                    LC823450_DMA_SRCWIDTH_WORD |
                    LC823450_DMA_DSTWIDTH_WORD,
                    (uint32_t)BUF_ACCESS('J'), (uint32_t)ptr, n / 4);

  lc823450_dmastart(_hrxdma,
                    _i2s_rxdma_callback,
                    &_sem_rxdma);

  _i2s_semtake(&_sem_rxdma);

  /* Invoke the callback handler */

  callback(dev, apb, arg, 0);
  return OK;
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
 * Name: lc823450_i2s_txsamplerate
 ****************************************************************************/

static uint32_t lc823450_i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  /* Change SSRC FSI rate */

  /* Stop/Reset/Unreset SSRC */

  putreg32(0, SSRC_MODE);
  putreg32(0, SSRC_SRESETB);
  putreg32(1, SSRC_SRESETB);
  while (getreg32(SSRC_STATUS) & 0x1);

  /* Setup FSI */

  putreg32(rate << 13, SSRC_FSI);

  /* Restart SSRC */

  putreg32(0x1, SSRC_MODE);
  while (getreg32(SSRC_STATUS) != 0x1);

  return 0;
}

/****************************************************************************
 * Name: lc823450_i2s_txdatawidth
 ****************************************************************************/

static uint32_t lc823450_i2s_txdatawidth(struct i2s_dev_s *dev, int bits)
{
  return 0;
}

/****************************************************************************
 * Name: _i2s_isr
 ****************************************************************************/

static int _i2s_isr(int irq, FAR void *context, FAR void *arg)
{
  uint32_t status = getreg32(ABUFSTS1);
  uint32_t irqen0 = getreg32(ABUFIRQEN0);

  /* Check C Buffer Under Level */

  if ((irqen0 & ABUFIRQEN0_BULIRQEN('C')) && (status & ABUFSTS1_BULVL('C')))
    {
      /* Disable C Buffer Under Level IRQ */

      modifyreg32(ABUFIRQEN0, ABUFIRQEN0_BULIRQEN('C'), 0);

      /* post semaphore for the waiter */

      nxsem_post(&_sem_buf_under);
    }

  /* Check J Buffer Over Level */

  if ((irqen0 & ABUFIRQEN0_BOLIRQEN('J')) && (status & ABUFSTS1_BOLVL('J')))
    {
      /* Disable J Buffer Over Level IRQ */

      modifyreg32(ABUFIRQEN0, ABUFIRQEN0_BOLIRQEN('J'), 0);

      /* post semaphore for the waiter */

      nxsem_post(&_sem_buf_over);
    }

  return 0;
}

/****************************************************************************
 * Name: lc823450_i2s_send
 ****************************************************************************/

static int lc823450_i2s_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                             i2s_callback_t callback, void *arg,
                             uint32_t timeout)
{
  volatile uint32_t *ptr = (uint32_t *)&apb->samp[apb->curbyte];
  uint32_t n = apb->nbytes;
  uint32_t bufc_enabled;
  uint32_t decsel;

  DEBUGASSERT(0 < n);

  decsel = getreg32(AUDSEL) & AUDSEL_DECSEL;

  if (0 == getreg32(BUF_DTCAP('C')))
    {
      /* C buffer is empty. Start buffering by disabling access control */

      modifyreg32(ABUFACCEN, ABUFACCEN_CDCEN('C'), 0);
    }

  bufc_enabled = getreg32(ABUFACCEN) & ABUFACCEN_CDCEN('C');

  if (bufc_enabled)
    {
      /* Enable C Buffer Under Level IRQ */

      modifyreg32(ABUFIRQEN0, 0, ABUFIRQEN0_BULIRQEN('C'));

      /* Wait for Audio Buffer */

      _i2s_semtake(&_sem_buf_under);
    }

  if (0 == decsel && (n & 0x3))
    {
      auderr("** PCM data is not word-aligned (n=%d) ** \n", n);

      /* Set size to align on a word boundary */

      n &= ~0x3;

      if (0 == n)
        {
          goto out;
        }
    }

  /* Setup and start DMA for I2S */

  if (n & 0x3)
    {
      lc823450_dmasetup(_htxdma,
                        LC823450_DMA_SRCINC |
                        LC823450_DMA_SRCWIDTH_BYTE |
                        LC823450_DMA_DSTWIDTH_BYTE,
                        (uint32_t)ptr, (uint32_t)BUF_ACCESS('C'), n);
    }
  else
    {
      lc823450_dmasetup(_htxdma,
                        LC823450_DMA_SRCINC |
                        LC823450_DMA_SRCWIDTH_WORD |
                        LC823450_DMA_DSTWIDTH_WORD,
                        (uint32_t)ptr, (uint32_t)BUF_ACCESS('C'), n / 4);
    }

  lc823450_dmastart(_htxdma,
                    _i2s_txdma_callback,
                    &_sem_txdma);

  _i2s_semtake(&_sem_txdma);

#ifdef SHOW_BUFFERING
  if (0 == bufc_enabled)
    {
      audinfo("buffering (remain=%d) \n", getreg32(BUF_DTCAP('C')));
    }
#endif

  /* Start C Buffer */

  if (0 == bufc_enabled && _i2s_tx_th_bytes < getreg32(BUF_DTCAP('C')))
    {
      modifyreg32(ABUFACCEN, 0, ABUFACCEN_CDCEN('C'));
    }

out:

  /* Invoke the callback handler */

  callback(dev, apb, arg, 0);
  return OK;
}

/****************************************************************************
 * Name: lc823450_dmic_enable
 ****************************************************************************/

static void lc823450_dmic_enable(void)
{
  /* Disable clock for DGMIC */

  modifyreg32(CLOCKEN, CLOCKEN_FCE_DGMIC, 0);

  /* ALC=off */

  modifyreg32(ALCCTL, 0x1, 0x0);

  /* DGMICCTL: XFDSP=direct,XEQT=XHPF=off,XMOD=75%,XFS64=64fs */

  putreg32(0x70000009, DGMICCTL);

  /* Enable clock for DGMIC */

  modifyreg32(CLOCKEN, 0, CLOCKEN_FCE_DGMIC);

  /* AUDSEL: PCMSEL=1 (dmic) */

  modifyreg32(AUDSEL, 0, AUDSEL_PCMSEL);

#if 1
  /* TODO: should be moved to another function */

  /* Enable clock for VOLUME(SP0) */

  modifyreg32(CLOCKEN, 0, CLOCKEN_FCE_VOLSP0);

  /* Audio Buffer Access Control: enable redirect E */

  modifyreg32(ABUFACCEN, 0, ABUFACCEN_RDCTEN('E'));

  /* Set the redirect source of I Buffer to E */

  modifyreg32(BUFCTL('I'), 0, BUFID('E') << 16);

  /* Enable CDCI */

  modifyreg32(ABUFACCEN, 0, ABUFACCEN_CDCEN('I'));

  /* Enable ASRC clock */

  modifyreg32(CLOCKEN, 0, CLOCKEN_FCE_ASRC);

  /* ASRC = 44.1k(in)/44.1k(out) */

  putreg32(44100 << 13, ASRC_FSI);
  putreg32(44100 << 0, ASRC_FSO);

  /* Adjust volume SP0 (+33dB) */

  putreg32(VOL_CONT_DIRECT |
           33 << 8 | 33,
           VOLSP0_CONT);

  audinfo("ASRC_FSIO=%d \n",  getreg32(ASRC_FSO));
  audinfo("DTCAP(I)=0x%x \n", getreg32(BUF_DTCAP('I')));
  audinfo("DTCAP(J)=0x%x \n", getreg32(BUF_DTCAP('J')));

  /* Start ASRC */

  putreg32(0x1, ASRC_MODE);
  while (getreg32(ASRC_STATUS) != 0x1);

  /* J Buffer : ACLTALN=1, ACLTEN=1 */

  modifyreg32(BUFCTL('J'), 0, 0x3 << 8);
#endif
}

/****************************************************************************
 * Name: lc823450_i2s_mp3dec
 ****************************************************************************/

static void lc823450_i2s_mp3dec(bool enable)
{
  if (enable)
    {
      modifyreg32(AUDSEL, 0, AUDSEL_DECSEL);
      modifyreg32(CLOCKEN, 0, CLOCKEN_FCE_MP3DEC);
      putreg32(0x1, MP3DEC_ERRMODE);
    }
  else
    {
      modifyreg32(CLOCKEN, CLOCKEN_FCE_MP3DEC, 0);
      modifyreg32(AUDSEL, AUDSEL_DECSEL, 0);
    }
}

/****************************************************************************
 * Name: lc823450_i2s_beeptest
 ****************************************************************************/

#ifdef BEEP_TEST
static void lc823450_i2s_beeptest(void)
{
  /* Enable clock */

  modifyreg32(CLOCKEN, 0, CLOCKEN_FCE_BEEP);

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
  uint32_t base = 0;

  _setup_audio_pll(44100);

  /* Unreset Audio Buffer */

  putreg32(MRSTCNTEXT3_AUDIOBUF_RSTB,
           MRSTCNTEXT3);

  /* Enable clock to Audio Buffer */

  putreg32(MCLKCNTEXT3_AUDIOBUF_CLKEN,
           MCLKCNTEXT3);

  /* C Buffer : size=56KB, under level=55kB */

  putreg32(base, BUF_BASE('C'));
  putreg32(1024 * 56, BUF_SIZE('C'));
  base += 1024 * 56;
  putreg32(1024 * 55, BUF_ULVL('C'));

  /* Setup F Buffer : size=512B */

  putreg32(base, BUF_BASE('F'));
  putreg32(512, BUF_SIZE('F'));
  base += 512;

  /* Setup I Buffer : size=2KB (TODO) */

  putreg32(base, BUF_BASE('I'));
  putreg32(2048 * 2, BUF_SIZE('I'));
  base += (2048 * 2);

  /* Setup J Buffer: size=4KB, over level=1KB */

  putreg32(base, BUF_BASE('J'));
  putreg32(4096, BUF_SIZE('J'));
  base += 4096;
  putreg32(1024, BUF_OLVL('J'));

  /* Clear Audio Buffer */

  putreg32(0xffff, ABUFCLR);

  /* Access Enable */

  modifyreg32(ABUFACCEN,
              0,
              ABUFACCEN_RDCTEN('D') |
              ABUFACCEN_CDCEN('F')
              );

  /* Source of F Buffer is D */

  modifyreg32(BUFCTL('F'), 0, BUFID('D') << 16);

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

  /* Enable function clocks */

  putreg32(CLOCKEN_FCE_PCKGEN |
           CLOCKEN_FCE_PCMPS0 |
           CLOCKEN_FCE_VOLPS0 |
           CLOCKEN_FCE_MUTED |
           CLOCKEN_FCE_SSRC |
           CLOCKEN_FCE_VOLD,
           CLOCKEN);

  /* SSRC = 44.1k(in)/44.1k(out) */

  putreg32(44100 << 13, SSRC_FSI);
  putreg32(44100 << 0, SSRC_FSO);

  /* Start SSRC */

  putreg32(0x1, SSRC_MODE);
  while (getreg32(SSRC_STATUS) != 0x1);

  audinfo("DTCAP(C)=0x%08x \n", BUF_DTCAP('C'));
  audinfo("DTCAP(I)=0x%08x \n", BUF_DTCAP('I'));
  audinfo("DTCAP(J)=0x%08x \n", BUF_DTCAP('J'));

  /* Setup default tx threshold */

  _setup_tx_threshold(0);
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

#if 1
  /* NOTE: should be moved to another codec driver */

  lc823450_dmic_enable();
#endif

  _hrxdma = lc823450_dmachannel(DMA_CHANNEL_VIRTUAL);
  nxsem_init(&_sem_rxdma, 0, 0);
  nxsem_init(&_sem_buf_over, 0, 0);

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

/****************************************************************************
 * Name: up_audio_bufcapacity
 ****************************************************************************/

uint32_t up_audio_bufcapacity(void)
{
  return (100 * getreg32(BUF_DTCAP('C'))) / getreg32(BUF_SIZE('C'));
}

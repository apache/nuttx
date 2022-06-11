/****************************************************************************
 * arch/arm/src/lc823450/lc823450_sddrv_dep.c
 *
 *   Copyright (C) 2014-2015 ON Semiconductor. All rights reserved.
 *   Copyright 2014,2015,2016,2017,2018 Sony Video & Sound Products Inc.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
 *   Author: Nobutaka Toyoshima <Nobutaka.Toyoshima@jp.sony.com>
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

#include <string.h>
#include <time.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>

#include "arm_internal.h"
#include "lc823450_sddrv_type.h"
#include "lc823450_sddrv_if.h"
#include "lc823450_dma.h"
#include "lc823450_gpio.h"
#include "lc823450_syscontrol.h"
#include "lc823450_timer.h"

#ifdef CONFIG_LC823450_SDC_DMA
#  include "lc823450_dma.h"
#endif /* CONFIG_LC823450_SDC_DMA */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SDIF0_BASE  (0x4004A000)
#define SDIF1_BASE  (0x4004B000)

#define DEBUG_PRINT(fmt, args...)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_LC823450_SDC_DMA
static DMA_HANDLE _hrdma[2];
static sem_t      _sem_rwait[2];
static DMA_HANDLE _hwdma[2];
static sem_t      _sem_wwait[2];
#endif /* CONFIG_LC823450_SDC_DMA */

static uint64_t _sddep_timeout = (10 * 100); /* 10sec (in tick) */

#ifndef CONFIG_HOTPLUG_SDC
extern void sdif_powerctrl(bool);
#endif

/****************************************************************************
 * Name: _get_ch_from_cfg
 ****************************************************************************/

static int _get_ch_from_cfg(struct sddrcfg_s *cfg)
{
  int ch = -1;
  switch (cfg->regbase)
    {
      case SDIF0_BASE:
        ch = 0;
        break;

      case SDIF1_BASE:
        ch = 1;
        break;

      default:
        DEBUGASSERT(false);
    }

  return ch;
}

/****************************************************************************
 * Name: dma_callback
 ****************************************************************************/

#ifdef CONFIG_LC823450_SDC_DMA
static void dma_callback(DMA_HANDLE hdma, void *arg, int result)
{
  sem_t *waitsem = (sem_t *)arg;
  nxsem_post(waitsem);
}
#endif /* CONFIG_LC823450_SDC_DMA */

/****************************************************************************
 * Name: _sddep_semtake
 ****************************************************************************/

static int _sddep_semtake(sem_t *sem)
{
  return nxsem_wait_uninterruptible(sem);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sddep0_hw_init
 ****************************************************************************/

SINT_T sddep0_hw_init(struct sddrcfg_s *cfg)
{
  irqstate_t flags = enter_critical_section();

  /* set COREVLT to 1 (i.e. 1.2v)
   * set MMCVLT0 to 1.8v
   * set EMMC
   */

  modifyreg32(SDCTL,
              0,
              SDCTL_COREVLT | SDCTL_MMCVLT0_18V | SDCTL_SDMMC0_MMC);

  /* pull-up SDCMD0/SDAT00-03 */

  modifyreg32(PUDCNT6, 0, (1UL << 2) | (1UL << 4));

  /* enable clock and unreset (SDIF0) */

  modifyreg32(MCLKCNTEXT1, 0, MCLKCNTEXT1_SDIF0_CLKEN);
  modifyreg32(MRSTCNTEXT1, 0, MRSTCNTEXT1_SDIF0_RSTB);

  leave_critical_section(flags);
  return 0;
}

/****************************************************************************
 * Name: sddep1_hw_init
 ****************************************************************************/

#ifdef CONFIG_LC823450_SDIF_SDC
SINT_T sddep1_hw_init(struct sddrcfg_s *cfg)
{
  int i;

  /* wait 15ms */

  nxsig_usleep(15000);

  irqstate_t flags = enter_critical_section();

  /* pull up SDCMD1/SDDATA10-13 which correspond to GPIO23-27
   * NOTE: SDCLK1 is not changed (i.e. none)
   */

  for (i = 3; i <= 7; i++)
    {
      lc823450_gpio_config(GPIO_PORT2 | (GPIO_PIN0 + i) |
                           GPIO_MODE_INPUT | GPIO_PULLUP);
    }

  /* enable clock and unreset (SDIF1) */

  modifyreg32(MCLKCNTEXT1, 0, MCLKCNTEXT1_SDIF1_CLKEN);
  modifyreg32(MRSTCNTEXT1, 0, MRSTCNTEXT1_SDIF1_RSTB);

  leave_critical_section(flags);
  return 0;
}
#endif /* CONFIG_LC823450_SDIF_SDC */

/****************************************************************************
 * Name: sddep0_hw_exit
 ****************************************************************************/

SINT_T sddep0_hw_exit(struct sddrcfg_s *cfg)
{
  irqstate_t flags = enter_critical_section();

  /* disable clock and reset (SDIF0) */

  modifyreg32(MCLKCNTEXT1, MCLKCNTEXT1_SDIF0_CLKEN, 0);
  modifyreg32(MRSTCNTEXT1, MRSTCNTEXT1_SDIF0_RSTB, 0);
  leave_critical_section(flags);
  return 0;
}

/****************************************************************************
 * Name: sddep1_hw_exit
 ****************************************************************************/

#ifdef CONFIG_LC823450_SDIF_SDC
SINT_T sddep1_hw_exit(struct sddrcfg_s *cfg)
{
  irqstate_t flags = enter_critical_section();

  /* pull down SDCMD1/SDDATA10-13 which correspond to GPIO23-27
   * NOTE: SDCLK1 is not changed (i.e. none)
   */

  int i;
  for (i = 3; i <= 7; i++)
    {
      lc823450_gpio_config(GPIO_PORT2 | (GPIO_PIN0 + i) |
                           GPIO_MODE_INPUT | GPIO_PULLDOWN);
    }

  /* SD ch1 power off */

#ifndef CONFIG_HOTPLUG_SDC
  sdif_powerctrl(false);
#endif

  /* disable clock and reset (SDIF1) */

  modifyreg32(MCLKCNTEXT1, MCLKCNTEXT1_SDIF1_CLKEN, 0);
  modifyreg32(MRSTCNTEXT1, MRSTCNTEXT1_SDIF1_RSTB, 0);

#ifdef CONFIG_LC823450_SDC_UHS1
  /* GPIO06=L 2.v */

  lc823450_gpio_config(GPIO_PORT0 | GPIO_PIN6 |
                       GPIO_MODE_OUTPUT | GPIO_VALUE_ZERO);
#endif

  leave_critical_section(flags);
  return 0;
}
#endif /* CONFIG_LC823450_SDIF_SDC */

/****************************************************************************
 * Name: sddep_voltage_switch
 ****************************************************************************/

void sddep_voltage_switch(struct sddrcfg_s *cfg)
{
#ifdef CONFIG_LC823450_SDC_UHS1
  /* GPIO06=H 1.8v */

  lc823450_gpio_config(GPIO_PORT0 | GPIO_PIN6 |
                       GPIO_MODE_OUTPUT | GPIO_VALUE_ONE);
  nxsig_usleep(200 * 1000);
#endif
}

/****************************************************************************
 * Name: sddep_os_init
 ****************************************************************************/

SINT_T sddep_os_init(struct sddrcfg_s *cfg)
{
  int ch = _get_ch_from_cfg(cfg);

#ifdef CONFIG_LC823450_SDC_DMA
  _hrdma[ch] = lc823450_dmachannel(DMA_CHANNEL_VIRTUAL);
  nxsem_init(&_sem_rwait[ch], 0, 0);
  _hwdma[ch] = lc823450_dmachannel(DMA_CHANNEL_VIRTUAL);
  nxsem_init(&_sem_wwait[ch], 0, 0);
#endif /* CONFIG_LC823450_SDC_DMA */
  return 0;
}

/****************************************************************************
 * Name: sddep_os_exit
 ****************************************************************************/

SINT_T sddep_os_exit(struct sddrcfg_s *cfg)
{
  return 0;
}

/****************************************************************************
 * Name: sddep_set_clk
 ****************************************************************************/

void sddep_set_clk(struct sddrcfg_s *cfg)
{
  if (cfg->clkdiv == 1)
    {
      DEBUG_PRINT("clock div = 1\n");
    }

  DEBUG_PRINT("clock = %d Hz\n", cfg->sysclk / cfg->clkdiv);
}

/****************************************************************************
 * Name: sddep_wait
 ****************************************************************************/

SINT_T sddep_wait(UI_32 ms, struct sddrcfg_s *cfg)
{
#ifdef CONFIG_HRT_TIMER
  up_hrttimer_usleep(ms * 1000);
#else
  if (1 == ms)
    {
      up_udelay(1000);
    }
  else
    {
      nxsig_usleep(ms * 1000);
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: sddep_set_timeout
 ****************************************************************************/

uint64_t sddep_set_timeout(uint64_t t)
{
  uint64_t ret = _sddep_timeout;
  _sddep_timeout = t;
  return ret;
}

/****************************************************************************
 * Name: sddep_wait_status
 ****************************************************************************/

SINT_T sddep_wait_status(UI_32 req_status, UI_32 *status,
                         struct sddrcfg_s *cfg)
{
  clock_t tick0 = clock_systime_ticks();
  int ret = 0;

  while (1)
    {
      clock_t tick1 = clock_systime_ticks();
      *status = sdif_get_status(cfg->regbase);
      if (req_status & (*status))
        {
          break;
        }

      if ((tick1 - tick0) > _sddep_timeout)
        {
          ret = -100;
          break;
        }

      sched_yield();
    }

  return ret;
}

/****************************************************************************
 * Name: sddep_read
 ****************************************************************************/

SINT_T sddep_read(void *src, void *dst, UI_32 size, SINT_T type,
                  struct sddrcfg_s *cfg)
{
#ifdef CONFIG_LC823450_SDC_DMA
  int ch = _get_ch_from_cfg(cfg);
  switch (type)
    {
      case SDDR_RW_INC_WORD:
      case SDDR_RW_NOINC_WORD:
        lc823450_dmasetup(_hrdma[ch],
                          LC823450_DMA_SRCWIDTH_WORD |
                          LC823450_DMA_DSTWIDTH_WORD |
                          (type == SDDR_RW_INC_WORD ?
                                   LC823450_DMA_DSTINC : 0),
                          (uint32_t)src, (uint32_t)dst, size / 4);
        break;

      case SDDR_RW_INC_HWORD:
      case SDDR_RW_NOINC_HWORD:
        lc823450_dmasetup(_hrdma[ch],
                          LC823450_DMA_SRCWIDTH_WORD |
                          LC823450_DMA_DSTWIDTH_HWORD |
                          (type == SDDR_RW_INC_HWORD ?
                                   LC823450_DMA_DSTINC : 0),
                          (uint32_t)src, (uint32_t)dst, size / 4);
        break;

      case SDDR_RW_INC_BYTE:
      case SDDR_RW_NOINC_BYTE:
        lc823450_dmasetup(_hrdma[ch],
                          LC823450_DMA_SRCWIDTH_WORD |
                          LC823450_DMA_DSTWIDTH_BYTE |
                          (type == SDDR_RW_INC_BYTE ?
                                   LC823450_DMA_DSTINC : 0),
                          (uint32_t)src, (uint32_t)dst, size / 4);
        break;
    }

  lc823450_dmastart(_hrdma[ch], dma_callback, &_sem_rwait[ch]);
  return _sddep_semtake(&_sem_rwait[ch]);
#else
  SINT_T i;
  UI_32 *p = (UI_32 *)src;
  UI_32 *buf = cfg->workbuf;

  for (i = 0; i < size / sizeof(UI_32); i++)
    {
      buf[i] = *p;
    }

  switch (type)
    {
      case SDDR_RW_INC_WORD:
      case SDDR_RW_INC_HWORD:
      case SDDR_RW_INC_BYTE:
        memcpy(dst, buf, size);
        break;

      case SDDR_RW_NOINC_WORD:
        for (i = 0; i < size / sizeof(UI_32); i++)
          {
            *(UI_32 *)dst = *(((UI_32 *)buf) + i);
          }
        break;

      case SDDR_RW_NOINC_HWORD:
        for (i = 0; i < size / sizeof(UI_16); i++)
          {
            *(UI_16 *)dst = *(((UI_16 *)buf) + i);
          }
        break;

      case SDDR_RW_NOINC_BYTE:
        for (i = 0; i < size / sizeof(UI_8); i++)
          {
            *(UI_8 *)dst = *(((UI_8 *)buf) + i);
          }
        break;

    default:
      return -100;
    }

  return 0;
#endif
}

/****************************************************************************
 * Name: sddep_write
 ****************************************************************************/

SINT_T sddep_write(void *src, void *dst, UI_32 size, SINT_T type,
                   struct sddrcfg_s *cfg)
{
#ifdef CONFIG_LC823450_SDC_DMA
  int ch = _get_ch_from_cfg(cfg);
  switch (type)
    {
      case SDDR_RW_INC_WORD:
      case SDDR_RW_NOINC_WORD:
        lc823450_dmasetup(_hwdma[ch],
                          LC823450_DMA_SRCWIDTH_WORD |
                          LC823450_DMA_DSTWIDTH_WORD |
                          (type == SDDR_RW_INC_WORD ?
                                   LC823450_DMA_SRCINC : 0),
                          (uint32_t)src, (uint32_t)dst, size / 4);
        break;

      case SDDR_RW_INC_HWORD:
      case SDDR_RW_NOINC_HWORD:
        lc823450_dmasetup(_hwdma[ch],
                          LC823450_DMA_SRCWIDTH_HWORD |
                          LC823450_DMA_DSTWIDTH_WORD |
                          (type == SDDR_RW_INC_HWORD ?
                                   LC823450_DMA_SRCINC : 0),
                          (uint32_t)src, (uint32_t)dst, size / 2);
            break;

      case SDDR_RW_INC_BYTE:
      case SDDR_RW_NOINC_BYTE:
        lc823450_dmasetup(_hwdma[ch],
                          LC823450_DMA_SRCWIDTH_BYTE |
                          LC823450_DMA_DSTWIDTH_WORD |
                          (type == SDDR_RW_INC_BYTE ?
                                   LC823450_DMA_SRCINC : 0),
                          (uint32_t)src, (uint32_t)dst, size);
        break;
    }

  lc823450_dmastart(_hwdma[ch], dma_callback, &_sem_wwait[ch]);
  return _sddep_semtake(&_sem_wwait[ch]);

#else
  SINT_T i;
  UI_32 *p = (UI_32 *)dst;
  UI_32 *buf = cfg->workbuf;

  switch (type)
    {
      case SDDR_RW_INC_WORD:
      case SDDR_RW_INC_HWORD:
      case SDDR_RW_INC_BYTE:
        memcpy(buf, src, size);
        break;

    case SDDR_RW_NOINC_WORD:
      for (i = 0; i < size / sizeof(UI_32); i++)
        {
          *(((UI_32 *)buf) + i) = *(UI_32 *)src;
        }
      break;

    case SDDR_RW_NOINC_HWORD:
      for (i = 0; i < size / sizeof(UI_16); i++)
        {
          *(((UI_16 *)buf) + i) = *(UI_16 *)src;
        }
      break;

    case SDDR_RW_NOINC_BYTE:
      for (i = 0; i < size / sizeof(UI_8); i++)
        {
          *(((UI_8 *)buf) + i) = *(UI_8 *)src;
        }
      break;

    default:
      return -100;
    }

  for (i = 0; i < size / sizeof(UI_32); i++)
    {
      *p = buf[i];
    }

  return 0;
#endif
}

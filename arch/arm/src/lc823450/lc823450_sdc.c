/****************************************************************************
 * arch/arm/src/lc823450/lc823450_sdc.c
 *
 *   Copyright (C) 2014-2015 ON Semiconductor. All rights reserved.
 *   Copyright 2014,2015,2016,2017 Sony Video & Sound Products Inc.
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
#include <sys/types.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/kthread.h>
#include <arch/board/board.h>

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "chip.h"
#include "arm_internal.h"
#include "lc823450_sddrv_if.h"
#include "lc823450_sdc.h"
#include "lc823450_syscontrol.h"
#include "lc823450_clockconfig.h"

#ifdef CONFIG_LED_ACCLED_SDIF
#  include <nuttx/led.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SDIF0_BASE  (0x4004A000)
#define SDIF1_BASE  (0x4004B000)

#define DET_TIME    (10000)    /* 10ms */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t _sdc_sem[2] =
{
  SEM_INITIALIZER(1),
  SEM_INITIALIZER(1)
};

static struct sddrcfg_s _sdch0;
static struct sddrcfg_s _sdch1;

static struct sddrcfg_s *_cfg[2] =
{
  &_sdch0,
  &_sdch1
};

static unsigned long _work0[512 / 4];
#ifdef CONFIG_LC823450_SDIF_SDC
static unsigned long _work1[512 / 4];
#endif

#ifdef CONFIG_LC823450_SDC_CACHE
static uint8_t   _sec_cache_enabled;
static uint32_t  _sec_cache[512 / 4];
static uint32_t  _sec_cache_add = 0xffffffff;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* ROM symbol address */

extern uint8_t cpu_ver;

extern SINT_T sddep0_hw_init(struct sddrcfg_s *);
extern SINT_T sddep0_hw_exit(struct sddrcfg_s *);
extern SINT_T sddep1_hw_init(struct sddrcfg_s *);
extern SINT_T sddep1_hw_exit(struct sddrcfg_s *);

extern SINT_T sddep_os_init(struct sddrcfg_s *);
extern SINT_T sddep_os_exit(struct sddrcfg_s *);
extern void   sddep_voltage_switch(struct sddrcfg_s *cfg);
extern void   sddep_set_clk(struct sddrcfg_s *);
extern SINT_T sddep_wait(UI_32, struct sddrcfg_s *);
extern SINT_T sddep_wait_status(UI_32 req, UI_32 *status,
                                struct sddrcfg_s *cfg);
extern SINT_T sddep_read(void *src, void *dst, UI_32 size, SINT_T type,
                         struct sddrcfg_s *cfg);
extern SINT_T sddep_write(void *src, void *dst, UI_32 size, SINT_T type,
                          struct sddrcfg_s *cfg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _sdc_semtake
 ****************************************************************************/

static int _sdc_semtake(sem_t *sem)
{
  return nxsem_wait_uninterruptible(sem);
}

/****************************************************************************
 * Name: _sdc_semgive
 ****************************************************************************/

static void _sdc_semgive(sem_t *sem)
{
  nxsem_post(sem);
}

/****************************************************************************
 * Name: _lc823450_sdc_support_trim
 ****************************************************************************/

static int _lc823450_sdc_support_trim(struct sddrcfg_s *cf)
{
  /* NOTE: to avoid conflicts, SDDR_SUPPORT_TRIM() macro is not used here */

  int ret = ((sddr_refmediatype(cf) == SDDR_MEDIA_TYPE_MMC) ?
             (((cf)->ex.mmc.extcsd_sec_feature_support & (1UL << 4)) ?
              1 : 0) : 0);
  return ret;
}

/****************************************************************************
 * Name: lc823450_sdc_access_led
 ****************************************************************************/

#ifdef CONFIG_LED_ACCLED_SDIF
static void lc823450_sdc_access_led(uint32_t ch, unsigned long sector)
{
  if (ch == 0)
    {
      if (sector >= CONFIG_MTD_CP_STARTBLOCK)
        {
          led_start_accled(false);
        }
    }
  else
    {
      led_start_accled(false);
    }
}
#else
#  define lc823450_sdc_access_led(a,b)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_sdc_clearcardinfo
 ****************************************************************************/

int lc823450_sdc_clearcardinfo(uint32_t ch)
{
  int ret;

  mcinfo("++++ start\n");

  ret = _sdc_semtake(&_sdc_sem[ch]);
  if (ret < 0)
    {
      return ret;
    }

  ret = sddr_clearcardinfo(_cfg[ch]);

#ifdef CONFIG_LC823450_SDC_CACHE
  if (ch)
    {
      _sec_cache_enabled = 0;
      _sec_cache_add = 0xffffffff; /* invalid */
    }
#endif

  _sdc_semgive(&_sdc_sem[ch]);
  mcinfo("---- end ret=%d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: lc823450_sdc_initialize
 ****************************************************************************/

int lc823450_sdc_initialize(uint32_t ch)
{
  int ret;

  /* Only ES2 is supported */

  DEBUGASSERT(1 == cpu_ver);

  struct sddrcfg_s *psd = _cfg[ch];

  psd->sysclk           = lc823450_get_systemfreq();
  psd->detecttime       = DET_TIME;

#ifdef CONFIG_LC823450_SDC_UHS1
  psd->setting          = SDDR_SD_SWITCH_18V;
#endif

  psd->deposinit        = sddep_os_init;
  psd->deposexit        = sddep_os_exit;
  psd->depsetclk        = sddep_set_clk;
  psd->depwait          = sddep_wait;
  psd->depwaitstatus    = sddep_wait_status;
  psd->depreaddata      = sddep_read;
  psd->depwritedata     = sddep_write;
  psd->depvoltageswitch = sddep_voltage_switch;

  switch (ch)
    {
      case 0:
        psd->dephwinit  = sddep0_hw_init;
        psd->dephwexit  = sddep0_hw_exit;
        psd->regbase    = SDIF0_BASE;
        psd->workbuf    = _work0;
        break;

#ifdef CONFIG_LC823450_SDIF_SDC
      case 1:
        psd->dephwinit  = sddep1_hw_init;
        psd->dephwexit  = sddep1_hw_exit;
        psd->regbase    = SDIF1_BASE;
        psd->workbuf    = _work1;
        break;
#endif

      default:
        DEBUGASSERT(false);
    }

  mcinfo("++++ start\n");

  ret = _sdc_semtake(&_sdc_sem[ch]);
  if (ret >= 0)
    {
      ret = sddr_initialize(_cfg[ch]);
      _sdc_semgive(&_sdc_sem[ch]);
      mcinfo("---- end ret=%d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: lc823450_sdc_finalize
 ****************************************************************************/

int lc823450_sdc_finalize(uint32_t ch)
{
  int ret;

  mcinfo("++++ start ch=%ld\n", ch);

  ret = _sdc_semtake(&_sdc_sem[ch]);
  if (ret >= 0)
    {
      ret = sddr_finalize(_cfg[ch]);
      _sdc_semgive(&_sdc_sem[ch]);
      mcinfo("---- end ret=%d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: lc823450_sdc_identifycard
 ****************************************************************************/

int lc823450_sdc_identifycard(uint32_t ch)
{
  int ret;

  mcinfo("++++ start\n");

  ret = _sdc_semtake(&_sdc_sem[ch]);
  if (ret < 0)
    {
      return ret;
    }

  ret = sddr_identifycard(_cfg[ch]);

#ifdef CONFIG_LC823450_SDC_CACHE
  if (ch)
    {
      _sec_cache_enabled = 0;
      _sec_cache_add = 0xffffffff; /* invalid */
    }
#endif

  _sdc_semgive(&_sdc_sem[ch]);
  mcinfo("---- end ret=%d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: lc823450_sdc_setclock
 ****************************************************************************/

int lc823450_sdc_setclock(uint32_t ch, uint32_t limitclk, uint32_t sysclk)
{
  int ret;

  mcinfo("++++ start ch=%ld limitClk=%ld sysClk=%ld\n",
         ch, limitclk, sysclk);

  ret = _sdc_semtake(&_sdc_sem[ch]);
  if (ret >= 0)
    {
      ret = sddr_setclock(limitclk, sysclk, _cfg[ch]);
      _sdc_semgive(&_sdc_sem[ch]);
      mcinfo("---- end ret=%d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: lc823450_sdc_refmediatype
 *
 * Returned Value:
 *   0(sd), 1(emmc)
 *
 ****************************************************************************/

int lc823450_sdc_refmediatype(uint32_t ch)
{
  int ret;

  mcinfo("++++ start\n");

  ret = _sdc_semtake(&_sdc_sem[ch]);
  if (ret >= 0)
    {
      ret = sddr_refmediatype(_cfg[ch]);
      _sdc_semgive(&_sdc_sem[ch]);
      mcinfo("---- end ret=%d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: lc823450_sdc_getcardsize
 ****************************************************************************/

int lc823450_sdc_getcardsize(uint32_t ch,
                             unsigned long *psecnum, unsigned long *psecsize)
{
  int ret;

  mcinfo("++++ start\n");

  ret = _sdc_semtake(&_sdc_sem[ch]);
  if (ret >= 0)
    {
      ret = sddr_getcardsize(psecnum, psecsize, _cfg[ch]);

      _sdc_semgive(&_sdc_sem[ch]);
      mcinfo("---- end ret=%d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: lc823450_sdc_readsector
 ****************************************************************************/

int lc823450_sdc_readsector(uint32_t ch,
                            unsigned long addr, unsigned short cnt,
                            void *pbuf, unsigned long type)
{
  int ret;
  int i = 0;

  ret = _sdc_semtake(&_sdc_sem[ch]);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_LC823450_SDC_LOG
  mcinfo("++++ start ch=%d, addr=%ld, cnt=%d\n", ch, addr, cnt);
#endif

#ifdef CONFIG_LC823450_SDC_CACHE
  if (ch && _sec_cache_enabled && 1 == cnt && addr == _sec_cache_add)
    {
      memcpy(pbuf, _sec_cache, sizeof(_sec_cache));
      goto errout_with_semaphore;
    }
#endif

  lc823450_sdc_access_led(ch, addr);

#ifdef CONFIG_SCHED_INSTRUMENTATION_IO
  sched_add_bi((uint64_t)cnt);
#endif

  for (i = 0; i < 5; i++)
    {
#ifdef CONFIG_LC823450_SDIF_PATCH
      ret = fixed_sddr_readsector(addr, cnt, pbuf, type, _cfg[ch]);
#else
      ret = sddr_readsector(addr, cnt, pbuf, type, _cfg[ch]);
#endif
      if (0 == ret)
        {
          break;
        }

      mcinfo("ret=%d ch=%" PRId32 " add=%ld cnt=%d i=%d\n",
             ret, ch, addr, cnt, i);
    }

#ifdef CONFIG_LC823450_SDC_CACHE
  if (ch)
    {
      if (0 == addr)
        {
          uint8_t *p = (pbuf + 0x1c2); /* partition id */
          if (0x7 == *p)
            {
              mcinfo("exFAT (NTFS) detected\n");
              _sec_cache_enabled = 1;
            }
        }

      if (_sec_cache_enabled)
        {
          if (1 == cnt && 0 == ret)
            {
              memcpy(_sec_cache, pbuf, sizeof(_sec_cache));
              _sec_cache_add = addr;
            }
          else
            {
              _sec_cache_add = 0xffffffff; /* invalid */
            }
        }
    }

errout_with_semaphore:
#endif
  _sdc_semgive(&_sdc_sem[ch]);

  mcinfo("----  end ret=%d\n", ret);
  return ret;
}

/****************************************************************************
 * name: lc823450_sdc_writesector
 ****************************************************************************/

int lc823450_sdc_writesector(uint32_t ch,
                             unsigned long addr, unsigned short cnt,
                             void *pbuf, unsigned long type)
{
  int ret;

  ret = _sdc_semtake(&_sdc_sem[ch]);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_LC823450_SDC_LOG
  mcinfo("++++ start ch=%d, addr=%ld, cnt=%d\n", ch, addr, cnt);
#endif

#ifdef CONFIG_LC823450_SDC_CACHE
  if (1 == ch && _sec_cache_enabled)
    {
      _sec_cache_add = 0xffffffff; /* invalid */
    }
#endif

  lc823450_sdc_access_led(ch, addr);

#ifdef CONFIG_SCHED_INSTRUMENTATION_IO
  sched_add_bo((uint64_t)cnt);
#endif

  ret = sddr_writesector(addr, cnt, pbuf, type, _cfg[ch]);

  if (0 > ret)
    {
      mcinfo("ret=%d ch=%" PRId32 " add=%ld cnt=%d\n", ret, ch, addr, cnt);
    }

  _sdc_semgive(&_sdc_sem[ch]);

  mcinfo("----  end ret=%d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: lc823450_sdc_checktrim
 ****************************************************************************/

int lc823450_sdc_checktrim(uint32_t ch)
{
  return _lc823450_sdc_support_trim(_cfg[ch]);
}

/****************************************************************************
 * Name: lc823450_sdc_trimsector
 ****************************************************************************/

int lc823450_sdc_trimsector(uint32_t ch, unsigned long addr,
                            unsigned short cnt)
{
  int ret;

  ret = _sdc_semtake(&_sdc_sem[ch]);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_LC823450_SDC_LOG
  mcinfo("++++ start ch=%d, addr=%ld, cnt=%d\n", ch, addr, cnt);
#endif

  lc823450_sdc_access_led(ch, addr);

#ifdef CONFIG_SCHED_INSTRUMENTATION_IO
  sched_add_bt((uint64_t)cnt);
#endif

  ret = sddr_eraseseq(0x00000001, addr, cnt, _cfg[ch]);
  if (0 > ret)
    {
      mcinfo("ret=%d ch=%" PRId32 " add=%ld cnt=%d\n", ret, ch, addr, cnt);
    }

  _sdc_semgive(&_sdc_sem[ch]);

  mcinfo("----  end ret=%d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: lc823450_sdc_cachectl
 ****************************************************************************/

int lc823450_sdc_cachectl(uint32_t ch, int ctrl)
{
  int ret;

  mcinfo("++++ ch=%" PRId32 ", ctrl=%d\n", ch, ctrl);

  ret = _sdc_semtake(&_sdc_sem[ch]);
  if (ret >= 0)
    {
      ret = sddr_cachectrl(ctrl, _cfg[ch]);

      _sdc_semgive(&_sdc_sem[ch]);
      mcinfo("----  end ret=%d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: lc823450_sdc_changespeedmode
 ****************************************************************************/

int lc823450_sdc_changespeedmode(uint32_t ch, int mode)
{
  int ret;

  mcinfo("++++ ch=%" PRId32 ", mode=%d\n", ch, mode);

  ret = _sdc_semtake(&_sdc_sem[ch]);
  if (ret < 0)
    {
      return ret;
    }

  ret = sddr_changespeedmode(mode, _cfg[ch]);

  if (0 == ret)
    {
      switch (mode)
        {
          case 1: /* High Speed */
            modifyreg32(SDCTL,
                        SDCTL_ACSMODE0_MASK << (ch * 8),
                        SDCTL_ACSMODE0_HS << (ch * 8));
            break;

          case 4: /* DDR */
            modifyreg32(SDCTL,
                        SDCTL_ACSMODE0_MASK << (ch * 8),
                        SDCTL_ACSMODE0_MMCDDR << (ch * 8));
            break;
        }
    }

  _sdc_semgive(&_sdc_sem[ch]);
  mcinfo("----  end ret=%d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: lc823450_sdc_getcid
 ****************************************************************************/

int lc823450_sdc_getcid(uint32_t ch, char *cidstr, int length)
{
  uint8_t cid[16];
  int ret;

  mcinfo("++++ ch=%" PRId32 "\n", ch);

  ret = _sdc_semtake(&_sdc_sem[ch]);
  if (ret < 0)
    {
      return ret;
    }

  ret = sddr_getcid((UI_32 *)cid, _cfg[ch]);

  if (0 == ret && length >= (2 * sizeof(cid) + 1))
    {
      int i;

      for (i = 15; i >= 0; i--)
        {
          snprintf(cidstr, 3, "%02x", cid[i]);
          cidstr += 2;
        }

      *cidstr = '\0';
    }

  _sdc_semgive(&_sdc_sem[ch]);
  mcinfo("----  end ret=%d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: lc823450_sdc_locked
 ****************************************************************************/

int lc823450_sdc_locked(void)
{
  int val;
  int ret;
  int i;

  ret = 0;

  for (i = 0; i < 2; i++)
    {
      nxsem_get_value(&_sdc_sem[i], &val);
      if (1 != val)
        {
          ret = 1;
          break;
        }
    }

  return ret;
}

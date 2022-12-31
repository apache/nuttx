/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_adc.c
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
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <arch/types.h>
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <arch/chip/scu.h>
#include <arch/chip/adc.h>

#include "chip.h"
#include "arm_internal.h"
#include "hardware/cxd56_adc.h"
#include "hardware/cxd56_scuseq.h"
#include "cxd56_clock.h"
#include "cxd56_adc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_CXD56_HPADC0_FSIZE
#define CONFIG_CXD56_HPADC0_FSIZE 64
#endif
#ifndef CONFIG_CXD56_HPADC1_FSIZE
#define CONFIG_CXD56_HPADC1_FSIZE 64
#endif
#ifndef CONFIG_CXD56_LPADC0_FSIZE
#define CONFIG_CXD56_LPADC0_FSIZE 16
#endif
#ifndef CONFIG_CXD56_LPADC1_FSIZE
#define CONFIG_CXD56_LPADC1_FSIZE 16
#endif
#ifndef CONFIG_CXD56_LPADC2_FSIZE
#define CONFIG_CXD56_LPADC2_FSIZE 16
#endif
#ifndef CONFIG_CXD56_LPADC3_FSIZE
#define CONFIG_CXD56_LPADC3_FSIZE 16
#endif

#ifndef CONFIG_CXD56_HPADC0_OFFSET
#define CONFIG_CXD56_HPADC0_OFFSET 0
#endif
#ifndef CONFIG_CXD56_HPADC1_OFFSET
#define CONFIG_CXD56_HPADC1_OFFSET 0
#endif
#ifndef CONFIG_CXD56_LPADC0_OFFSET
#define CONFIG_CXD56_LPADC0_OFFSET 0
#endif
#ifndef CONFIG_CXD56_LPADC1_OFFSET
#define CONFIG_CXD56_LPADC1_OFFSET 0
#endif
#ifndef CONFIG_CXD56_LPADC2_OFFSET
#define CONFIG_CXD56_LPADC2_OFFSET 0
#endif
#ifndef CONFIG_CXD56_LPADC3_OFFSET
#define CONFIG_CXD56_LPADC3_OFFSET 0
#endif

#ifndef CONFIG_CXD56_HPADC0_GAIN
#define CONFIG_CXD56_HPADC0_GAIN 0
#endif
#ifndef CONFIG_CXD56_HPADC1_GAIN
#define CONFIG_CXD56_HPADC1_GAIN 0
#endif
#ifndef CONFIG_CXD56_LPADC0_GAIN
#define CONFIG_CXD56_LPADC0_GAIN 0
#endif
#ifndef CONFIG_CXD56_LPADC1_GAIN
#define CONFIG_CXD56_LPADC1_GAIN 0
#endif
#ifndef CONFIG_CXD56_LPADC2_GAIN
#define CONFIG_CXD56_LPADC2_GAIN 0
#endif
#ifndef CONFIG_CXD56_LPADC3_GAIN
#define CONFIG_CXD56_LPADC3_GAIN 0
#endif

#define ADC_BYTESPERSAMPLE   2
#define ADC_ELEMENTSIZE      0

/* Input Gain setting */

#define INPUT_GAIN(a, g1, g2) (((a) << 24) | ((g2) << 16) | ((g1) << 12))
#define INPUT_GAIN_MASK         INPUT_GAIN(3, 15, 15)
#define INPUT_GAIN_MINUS_6DB    INPUT_GAIN(2, 0, 0)
#define INPUT_GAIN_THROUGH      INPUT_GAIN(0, 0, 0)
#define INPUT_GAIN_PLUS_6DB     INPUT_GAIN(0, 4, 0)
#define INPUT_GAIN_PLUS_12DB    INPUT_GAIN(0, 12, 0)
#define INPUT_GAIN_PLUS_14DB    INPUT_GAIN(0, 4, 8)

#if defined(CONFIG_CXD56_HPADC0_INPUT_GAIN_M6DB)
#define HPADC0_INPUT_GAIN       INPUT_GAIN_MINUS_6DB
#elif defined(CONFIG_CXD56_HPADC0_INPUT_GAIN_6DB)
#define HPADC0_INPUT_GAIN       INPUT_GAIN_PLUS_6DB
#elif defined(CONFIG_CXD56_HPADC0_INPUT_GAIN_12DB)
#define HPADC0_INPUT_GAIN       INPUT_GAIN_PLUS_12DB
#elif defined(CONFIG_CXD56_HPADC0_INPUT_GAIN_14DB)
#define HPADC0_INPUT_GAIN       INPUT_GAIN_PLUS_14DB
#else
#define HPADC0_INPUT_GAIN       INPUT_GAIN_THROUGH
#endif

#if defined(CONFIG_CXD56_HPADC1_INPUT_GAIN_M6DB)
#define HPADC1_INPUT_GAIN       INPUT_GAIN_MINUS_6DB
#elif defined(CONFIG_CXD56_HPADC1_INPUT_GAIN_6DB)
#define HPADC1_INPUT_GAIN       INPUT_GAIN_PLUS_6DB
#elif defined(CONFIG_CXD56_HPADC1_INPUT_GAIN_12DB)
#define HPADC1_INPUT_GAIN       INPUT_GAIN_PLUS_12DB
#elif defined(CONFIG_CXD56_HPADC1_INPUT_GAIN_14DB)
#define HPADC1_INPUT_GAIN       INPUT_GAIN_PLUS_14DB
#else
#define HPADC1_INPUT_GAIN       INPUT_GAIN_THROUGH
#endif

typedef enum adc_ch
{
  CH0 = 0,    /* LPADC0 */
  CH1,        /* LPADC1 */
  CH2,        /* LPADC2 */
  CH3,        /* LPADC3 */
  CH4,        /* HPADC0 */
  CH5,        /* HPADC1 */
  CH_MAX,
} adc_ch_t;

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one ADC channel */

struct cxd56adc_dev_s
{
  adc_ch_t         ch;            /* adc channel number */
  struct seq_s     *seq;          /* sequencer */
  uint8_t          freq;          /* coefficient of adc sampling frequency */
  uint16_t         fsize;         /* SCU FIFO size */
  uint16_t         ofst;          /* offset */
  uint16_t         gain;          /* gain */
  uint8_t          fifomode;      /* fifo mode */
  struct scufifo_wm_s *wm;        /* water mark */
  struct math_filter_s *filter;   /* math filter */
  struct scuev_notify_s *notify;  /* notify */
  mutex_t          lock;          /* exclusive mutex */
  int              crefs;         /* reference count */
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int cxd56_adc_open(struct file *filep);
static int cxd56_adc_close(struct file *filep);
static ssize_t cxd56_adc_read(struct file *filep, char *buffer,
                              size_t len);
static int cxd56_adc_ioctl(struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC interface operations */

static const struct file_operations g_adcops =
{
  cxd56_adc_open,            /* open */
  cxd56_adc_close,           /* close */
  cxd56_adc_read,            /* read */
  NULL,                      /* write */
  NULL,                      /* seek */
  cxd56_adc_ioctl,           /* ioctl */
};

#if defined (CONFIG_CXD56_LPADC0) || defined (CONFIG_CXD56_LPADC0_1) || defined (CONFIG_CXD56_LPADC_ALL)
static struct cxd56adc_dev_s g_lpadc0priv =
{
  .ch     = CH0,
  .seq    = NULL,
  .freq   = CONFIG_CXD56_LPADC0_FREQ,
  .fsize  = CONFIG_CXD56_LPADC0_FSIZE,
  .ofst   = CONFIG_CXD56_LPADC0_OFFSET,
  .gain   = CONFIG_CXD56_LPADC0_GAIN,
  .wm     = NULL,
  .filter = NULL,
  .notify = NULL,
  .lock   = NXMUTEX_INITIALIZER,
  .crefs  = 0,
};
#endif

#if defined (CONFIG_CXD56_LPADC1) || defined (CONFIG_CXD56_LPADC0_1) || defined (CONFIG_CXD56_LPADC_ALL)
static struct cxd56adc_dev_s g_lpadc1priv =
{
  .ch     = CH1,
  .seq    = NULL,
  .freq   = CONFIG_CXD56_LPADC1_FREQ,
  .fsize  = CONFIG_CXD56_LPADC1_FSIZE,
  .ofst   = CONFIG_CXD56_LPADC1_OFFSET,
  .gain   = CONFIG_CXD56_LPADC1_GAIN,
  .wm     = NULL,
  .filter = NULL,
  .notify = NULL,
  .lock   = NXMUTEX_INITIALIZER,
  .crefs  = 0,
};
#endif

#if defined (CONFIG_CXD56_LPADC2) || defined (CONFIG_CXD56_LPADC_ALL)
static struct cxd56adc_dev_s g_lpadc2priv =
{
  .ch     = CH2,
  .seq    = NULL,
  .freq   = CONFIG_CXD56_LPADC2_FREQ,
  .fsize  = CONFIG_CXD56_LPADC2_FSIZE,
  .ofst   = CONFIG_CXD56_LPADC2_OFFSET,
  .gain   = CONFIG_CXD56_LPADC2_GAIN,
  .wm     = NULL,
  .filter = NULL,
  .notify = NULL,
  .lock   = NXMUTEX_INITIALIZER,
  .crefs  = 0,
};
#endif

#if defined (CONFIG_CXD56_LPADC3) || defined (CONFIG_CXD56_LPADC_ALL)
static struct cxd56adc_dev_s g_lpadc3priv =
{
  .ch     = CH3,
  .seq    = NULL,
  .freq   = CONFIG_CXD56_LPADC3_FREQ,
  .fsize  = CONFIG_CXD56_LPADC3_FSIZE,
  .ofst   = CONFIG_CXD56_LPADC3_OFFSET,
  .gain   = CONFIG_CXD56_LPADC3_GAIN,
  .wm     = NULL,
  .filter = NULL,
  .notify = NULL,
  .lock   = NXMUTEX_INITIALIZER,
  .crefs  = 0,
};
#endif

#ifdef CONFIG_CXD56_HPADC0
static struct cxd56adc_dev_s g_hpadc0priv =
{
  .ch     = CH4,
  .seq    = NULL,
  .freq   = CONFIG_CXD56_HPADC0_FREQ,
  .fsize  = CONFIG_CXD56_HPADC0_FSIZE,
  .ofst   = CONFIG_CXD56_HPADC0_OFFSET,
  .gain   = CONFIG_CXD56_HPADC0_GAIN,
  .wm     = NULL,
  .filter = NULL,
  .notify = NULL,
  .lock   = NXMUTEX_INITIALIZER,
  .crefs  = 0,
};
#endif

#ifdef CONFIG_CXD56_HPADC1
static struct cxd56adc_dev_s g_hpadc1priv =
{
  .ch     = CH5,
  .seq    = NULL,
  .freq   = CONFIG_CXD56_HPADC1_FREQ,
  .fsize  = CONFIG_CXD56_HPADC1_FSIZE,
  .ofst   = CONFIG_CXD56_HPADC1_OFFSET,
  .gain   = CONFIG_CXD56_HPADC1_GAIN,
  .wm     = NULL,
  .filter = NULL,
  .notify = NULL,
  .lock   = NXMUTEX_INITIALIZER,
  .crefs  = 0,
};
#endif

static bool adc_active[CH_MAX] =
{
    false, false, false, false, false, false
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: set_ofstgain
 *
 * Description:
 *   Set parameters for offset/gain.
 *
 ****************************************************************************/

static int set_ofstgain(struct cxd56adc_dev_s *priv)
{
  int ret = OK;
  uint32_t addr;

  if (priv->ch == CH5)
    {
      addr = SCUSEQ_ADC_MATH_PROC_OFST_GAIN(priv->ch + 1);
    }
  else
    {
      addr = SCUSEQ_ADC_MATH_PROC_OFST_GAIN(priv->ch);
    }

  if ((priv->ofst > 0) || (priv->gain > 0))
    {
      putreg32(priv->gain << 16 | priv->ofst, addr);
      ret = seq_ioctl(priv->seq, 0, SCUIOC_SETOGADJUST, 0);
      if (ret < 0)
        {
          aerr("SETOGADJUST failed. %d\n", ret);
          return ret;
        }
    }
  else
    {
      putreg32(0x08000000, addr);
      ret = seq_ioctl(priv->seq, 0, SCUIOC_CLROGADJUST, 0);
      if (ret < 0)
        {
          aerr("CLROGADJUST failed. %d\n", ret);
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: adc_start
 *
 * Description:
 *   Start ADC channel.
 *
 ****************************************************************************/

static int adc_start(adc_ch_t ch, uint8_t freq, struct seq_s *seq,
        int fsize, int fifomode,
        struct scufifo_wm_s *wm,
        struct math_filter_s *filter,
        struct scuev_notify_s *notify)
{
  uint32_t *addr;
  uint32_t val;
  uint32_t mask;
  uint32_t div;
  int ret;

  if (adc_active[ch])
    {
      return OK;
    }

  ret = seq_ioctl(seq, 0, SCUIOC_SETFIFO, fsize);
  if (ret < 0)
    {
      aerr("SETFIFO failed. %d\n", ret);
      return ret;
    }

  ret = seq_ioctl(seq, 0, SCUIOC_SETFIFOMODE, fifomode);
  if (ret < 0)
    {
      aerr("SETFIFOMODE failed. %d\n", ret);
      return ret;
    }

  if (wm)
    {
      ret = seq_ioctl(seq, 0, SCUIOC_SETWATERMARK, (unsigned long)wm);
      if (ret < 0)
        {
          aerr("SETWATERMARK failed. %d\n", ret);
          return ret;
        }
    }

  if (filter)
    {
      ret = seq_ioctl(seq, 0, SCUIOC_SETFILTER, (unsigned long)filter);
      if (ret < 0)
        {
          aerr("SETFILTER failed. %d\n", ret);
          return ret;
        }
    }

  if (notify)
    {
      ret = seq_ioctl(seq, 0, SCUIOC_SETNOTIFY, (unsigned long)notify);
      if (ret < 0)
        {
          aerr("SETNOTIFY failed. %d\n", ret);
          return ret;
        }
    }

  if (ch <= CH3)
    {
      /* LPADC.A1 LPADC_CH : todo: GPS ch */

      val = ch;
#ifdef CONFIG_CXD56_LPADC0_1
      val = 4;
#endif
#ifdef CONFIG_CXD56_LPADC_ALL
      val = 5;
#endif
      putreg32(val, SCUADCIF_LPADC_A1);

      /* LPADC_D1/D4/D5/D6 ratio */

      div = (freq <= 15) ? freq : 15;
      addr = (ch == CH0) ? (uint32_t *)SCUADCIF_LPADC_D1 :
                           (uint32_t *)(SCUADCIF_LPADC_D4 + 4 * (ch - CH1));
      mask = 0x00000f00;
      val = getreg32(addr) & ~mask;
      val |= (div << 8);
      putreg32(val, addr);

      /* LPADC_AT1 */

      mask = 0x000000ff;
      val = getreg32(SCUADCIF_LPADC_AT1) & ~mask;
      val |= 0x00000004;
      putreg32(val, SCUADCIF_LPADC_AT1);

      /* LPADC_AT0 */

      mask = 0x00000001;
      val = getreg32(SCUADCIF_LPADC_AT0) & ~mask;
      putreg32(val, SCUADCIF_LPADC_AT0);

      /* power and clock */

      div = (freq <= 15) ? 0 : freq - 15;
      cxd56_lpadc_clock_enable(div);

      /* LPADC.A0 */

      putreg32(1, SCUADCIF_LPADC_A0);

      /* LPADC.D0 */

      putreg32(1, SCUADCIF_LPADC_D0);
    }
  else
    {
      /* HPADC.AC0 */

#ifndef CONFIG_CXD56_SCU_32K
#ifdef CONFIG_CXD56_SCU_RCOSC
      val = 0x00000001;
#else
      uint32_t xoscfreq = cxd56_get_xosc_clock();
      if (xoscfreq <= 24000000)
        {
          val = 0x00000000;
        }
      else if (xoscfreq <= 30000000)
        {
          val = 0x00000010;
        }
      else if (xoscfreq <= 45000000)
        {
          val = 0x00000020;
        }
      else
        {
          val = 0x00000030;
        }

#endif
      putreg32(val, SCUADCIF_HPADC_AC0);
#endif

      /* HPADC.AC1 */

      putreg32(1, SCUADCIF_HPADC_AC1);

      /* HPADCn.A0 */

      addr = (ch == CH4) ? (uint32_t *)SCUADCIF_HPADC0_A0 :
                           (uint32_t *)SCUADCIF_HPADC1_A0;
#ifdef CONFIG_CXD56_SCU_32K
      val = 0x00000001;
#else
      val = 0x00000000;
#endif
      putreg32(val, addr);

      /* HPADC.DC : todo: HPADC0 16bit x 2 */

      /* HPADCn.AT0 */

      addr = (ch == CH4) ? (uint32_t *)SCUADCIF_HPADC0_AT0 :
                           (uint32_t *)SCUADCIF_HPADC1_AT0;
      mask = 0x000000f1;
      val = getreg32(addr) & ~mask;
      val |= 0x000000c0;
      putreg32(val, addr);

      /* power and clock */

      div = (freq <= 7) ? 0 : freq - 7;
      cxd56_hpadc_clock_enable(div);

      /* HPADC.A1 */

      addr = (ch == CH4) ? (uint32_t *)SCUADCIF_HPADC0_A1 :
                           (uint32_t *)SCUADCIF_HPADC1_A1;
      putreg32(7, addr);

      /* HPADC.A2 */

      addr = (ch == CH4) ? (uint32_t *)SCUADCIF_HPADC0_A2 :
                           (uint32_t *)SCUADCIF_HPADC1_A2;
      putreg32(1, addr);

      /* HPADC.A3 */

      addr = (ch == CH4) ? (uint32_t *)SCUADCIF_HPADC0_A3 :
                           (uint32_t *)SCUADCIF_HPADC1_A3;

      val = getreg32(addr) & ~INPUT_GAIN_MASK;
      val |= (ch == CH4) ? HPADC0_INPUT_GAIN : HPADC1_INPUT_GAIN;
      putreg32(val, addr);

      /* HPADC.D0 */

      addr = (ch == CH4) ? (uint32_t *)SCUADCIF_HPADC0_D0 :
                           (uint32_t *)SCUADCIF_HPADC1_D0;
      putreg32(1, addr);

      /* HPADC.D1 */

      div = (freq <= 7) ? freq : 7;
      addr = (ch == CH4) ? (uint32_t *)SCUADCIF_HPADC0_D1 :
                           (uint32_t *)SCUADCIF_HPADC1_D1;
      mask = 0x00000700;
      val = getreg32(addr) & ~mask;
      val |= (div << 8);
      putreg32(val, addr);
    }

  ret = seq_ioctl(seq, 0, SCUIOC_START, 0);

  if (!ret)
    {
      if (ch <= CH3)
        {
          /* LPADC.D2 */

          mask = 0x1 << ch;
          val = getreg32(SCUADCIF_LPADC_D2) & ~mask;
          val |= mask;
          putreg32(val, SCUADCIF_LPADC_D2);
        }
      else
        {
          /* HPADC.D2 */

          addr = (ch == CH4) ? (uint32_t *)SCUADCIF_HPADC0_D2 :
                               (uint32_t *)SCUADCIF_HPADC1_D2;
          putreg32(1, addr);
        }

      adc_active[ch] = true;
    }
  else
    {
      /* todo:err case */
    }

  return ret;
}

/****************************************************************************
 * Name: adc_stop
 *
 * Description:
 *   Stop ADC channel.
 *
 ****************************************************************************/

static int adc_stop(adc_ch_t ch, struct seq_s *seq)
{
  uint32_t *addr;
  uint32_t val;
  uint32_t mask;
  int i;
  bool is_clockdisable = true;

  if (!adc_active[ch])
    {
      return OK;
    }

  seq_ioctl(seq, 0, SCUIOC_STOP, 0);

  if (ch <= CH3)
    {
      /* LPADC.D2 */

      mask = 0x1 << ch;
      val = getreg32(SCUADCIF_LPADC_D2) & ~mask;
      putreg32(val, SCUADCIF_LPADC_D2);

      for (i = 0; i < CH4; i++)
        {
          if ((i != ch) && adc_active[i])
            {
              is_clockdisable = false;
              break;
            }
        }

      if (is_clockdisable)
        {
          cxd56_lpadc_clock_disable();
        }
    }
  else
    {
      /* HPADC.D2 */

      addr = (ch == CH4) ? (uint32_t *)SCUADCIF_HPADC0_D2 :
                           (uint32_t *)SCUADCIF_HPADC1_D2;
      putreg32(0, addr);

      if (((ch == CH4) && !adc_active[CH5]) ||
          ((ch == CH5) && !adc_active[CH4]))
        {
          cxd56_hpadc_clock_disable();
        }
    }

  adc_active[ch] = false;

  return OK;
}

/****************************************************************************
 * Name: adc_validcheck
 *
 * Description:
 *   Check ioctl command to pass to scu driver.
 *
 ****************************************************************************/

static bool adc_validcheck(int cmd)
{
  if ((cmd == SCUIOC_SETFILTER) ||
      (cmd == SCUIOC_SETNOTIFY) ||
      (cmd == SCUIOC_SETWATERMARK) ||
      (cmd == SCUIOC_SETFIFOMODE) ||
      (cmd == SCUIOC_DELFIFODATA))
    {
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: cxd56_adc_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int cxd56_adc_open(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  struct cxd56adc_dev_s *priv = inode->i_private;
  int ret = OK;
  int type;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->ch < CH_MAX);

  /* Increment reference counter */

  nxmutex_lock(&priv->lock);

  priv->crefs++;
  DEBUGASSERT(priv->crefs > 0);

  if (priv->crefs > 1)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  DEBUGASSERT(priv->seq == NULL);

  type = SCU_BUS_LPADC0 + priv->ch;

  /* Open sequencer */

  priv->seq = seq_open(SEQ_TYPE_NORMAL, type);
  if (!priv->seq)
    {
      nxmutex_unlock(&priv->lock);
      return -ENOENT;
    }

  /* Set sample data information to sequencer */

  seq_setsample(priv->seq, ADC_BYTESPERSAMPLE, 0, ADC_ELEMENTSIZE, false);

  /* Set offset/gain */

  ret = set_ofstgain(priv);
  if (ret < 0)
    {
      nxmutex_unlock(&priv->lock);
      return ret;
    }

  ainfo("open ch%d freq%d scufifo%d\n", priv->ch, priv->freq, priv->fsize);

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: cxd56_adc_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int cxd56_adc_close(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  struct cxd56adc_dev_s *priv = inode->i_private;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->seq != NULL);
  DEBUGASSERT(priv->ch < CH_MAX);

  /* Decrement reference counter */

  nxmutex_lock(&priv->lock);

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  if (priv->crefs > 0)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  /* Close sequencer */

  seq_close(priv->seq);
  priv->seq = NULL;

  if (priv->wm)
    {
      kmm_free(priv->wm);
      priv->wm = NULL;
    }

  if (priv->filter)
    {
      kmm_free(priv->filter);
      priv->filter = NULL;
    }

  if (priv->notify)
    {
      kmm_free(priv->notify);
      priv->notify = NULL;
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: cxd56_adc_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t cxd56_adc_read(struct file *filep, char *buffer,
                              size_t len)
{
  struct inode *inode = filep->f_inode;
  struct cxd56adc_dev_s *priv = inode->i_private;
  int ret = OK;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->seq != NULL);
  DEBUGASSERT(priv->ch < CH_MAX);

  len = len / ADC_BYTESPERSAMPLE * ADC_BYTESPERSAMPLE;
  ret = seq_read(priv->seq, 0, buffer, len);

  return ret;
}

/****************************************************************************
 * Name: cxd56_adc_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int cxd56_adc_ioctl(struct file *filep, int cmd,
                           unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  struct cxd56adc_dev_s *priv = inode->i_private;
  int ret = OK;
  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->seq != NULL);
  DEBUGASSERT(priv->ch < CH_MAX);

  switch (cmd)
    {
      case ANIOC_TRIGGER:
      case ANIOC_CXD56_START:
        ret = adc_start(priv->ch, priv->freq, priv->seq,
                priv->fsize, priv->fifomode,
                priv->wm, priv->filter, priv->notify);
        break;

      case ANIOC_CXD56_STOP:
        ret = adc_stop(priv->ch, priv->seq);
        break;

      case ANIOC_CXD56_FREQ:
        priv->freq = arg;
        break;

      case ANIOC_CXD56_FIFOSIZE:
        priv->fsize = arg;
        break;

      case SCUIOC_SETFIFOMODE:
        priv->fifomode = arg;
        break;

      case SCUIOC_SETWATERMARK:
        if (adc_active[priv->ch] == false) /* before start */
          {
            struct scufifo_wm_s *wm = (struct scufifo_wm_s *)arg;
            priv->wm = (struct scufifo_wm_s *)
              kmm_malloc(sizeof(struct scufifo_wm_s));
            *(priv->wm) = *wm;
          }
        else
          {
            ret = seq_ioctl(priv->seq, 0, cmd, arg);
          }
        break;

      case SCUIOC_SETFILTER:
        if (adc_active[priv->ch] == false) /* before start */
          {
            struct math_filter_s *filter = (struct math_filter_s *)arg;
            priv->filter = (struct math_filter_s *)
              kmm_malloc(sizeof(struct math_filter_s));
            *(priv->filter) = *filter;
          }
        else
          {
            ret = seq_ioctl(priv->seq, 0, cmd, arg);
          }
        break;

      case SCUIOC_SETNOTIFY:
        if (adc_active[priv->ch] == false) /* before start */
          {
            struct scuev_notify_s *notify = (struct scuev_notify_s *)arg;
            priv->notify = (struct scuev_notify_s *)
              kmm_malloc(sizeof(struct scuev_notify_s));
            *(priv->notify) = *notify;
          }
        else
          {
            ret = seq_ioctl(priv->seq, 0, cmd, arg);
          }
        break;

      case ANIOC_GET_NCHANNELS:
        {
          /* Return the number of configured channels */

          ret = 1;
        }
        break;

      default:
        {
          if (adc_validcheck(cmd))
            {
              /* Redirect SCU commands */

              ret = seq_ioctl(priv->seq, 0, cmd, arg);
            }
          else
            {
              aerr("Unrecognized cmd: %d\n", cmd);
              ret = -ENOTTY;
            }
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_adc_getinterval
 *
 * Description:
 *   get sampling interval of ADC.
 *
 * Input Parameters:
 *   bustype - SCU_BUS_LPADC0, SCU_BUS_LPADC1, SCU_BUS_LPADC2,
 *             SCU_BUS_LPADC3, SCU_BUS_HPADC0, SCU_BUS_HPADC1
 *   *interval - Sampling interval
 *   *adjust   - Adjustment value used for timestamp calculation
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cxd56_adc_getinterval(int adctype, uint32_t *interval, uint16_t *adjust)
{
  uint32_t intval = 0;
  uint32_t div = 0;

  DEBUGASSERT(interval != NULL && adjust != NULL);

  if ((adctype >= SCU_BUS_LPADC0) && (adctype <= SCU_BUS_LPADC3))
    {
      uint32_t chmodediv = 0;

      *adjust = 0;
#ifdef CONFIG_CXD56_LPADC0_1
      chmodediv = 1;
#endif
#ifdef CONFIG_CXD56_LPADC_ALL
      chmodediv = 2;
#endif

      if (adctype == SCU_BUS_LPADC0)
        {
#if defined (CONFIG_CXD56_LPADC0) || defined (CONFIG_CXD56_LPADC0_1) || defined (CONFIG_CXD56_LPADC_ALL)
          div = g_lpadc0priv.freq;
#endif
        }
      else if (adctype == SCU_BUS_LPADC1)
        {
#if defined (CONFIG_CXD56_LPADC1) || defined (CONFIG_CXD56_LPADC0_1) || defined (CONFIG_CXD56_LPADC_ALL)
          div = g_lpadc1priv.freq;
#endif
        }
      else if (adctype == SCU_BUS_LPADC2)
        {
#if defined (CONFIG_CXD56_LPADC2) || defined (CONFIG_CXD56_LPADC_ALL)
          div = g_lpadc2priv.freq;
#endif
        }
      else if (adctype == SCU_BUS_LPADC3)
        {
#if defined (CONFIG_CXD56_LPADC3) || defined (CONFIG_CXD56_LPADC_ALL)
          div = g_lpadc3priv.freq;
#endif
        }

      *interval = 1 << (chmodediv + div);
    }
  else if ((adctype == SCU_BUS_HPADC0) || (adctype == SCU_BUS_HPADC1))
    {
      *adjust = 1;
      if (adctype == SCU_BUS_HPADC0)
        {
#ifdef CONFIG_CXD56_HPADC0
          div = g_hpadc0priv.freq;
#endif
        }
      else if (adctype == SCU_BUS_HPADC1)
        {
#ifdef CONFIG_CXD56_HPADC1
          div = g_hpadc1priv.freq;
#endif
        }

#ifdef CONFIG_CXD56_SCU_32K
      *interval = 1 << div;
#else
      uint32_t freq = 0;
#ifdef CONFIG_CXD56_SCU_RCOSC
      freq = cxd56_get_rcosc_clock();
#else
      freq = cxd56_get_xosc_clock();
      if (freq <= 24000000)
        {
          freq /= 2;
        }
      else if (freq <= 30000000)
        {
          freq /= 3;
        }
      else if (freq <= 45000000)
        {
          freq /= 4;
        }
      else
        {
          freq /= 6;
        }

#endif
      if (freq > 0)
        {
          intval = (32768 << (2 + div + 1)) / freq;
          if (intval & 1)
            {
              *interval = (intval >> 1) + 1;
            }
          else
            {
              *interval = (intval >> 1);
            }
        }
#endif
    }
}

/****************************************************************************
 * Name: cxd56_adcinitialize
 *
 * Description:
 *   Initialize ADC channel for use with the upper_level ADC driver.
 *
 * Input Parameters:
 *   channel - adc channel number.
 *
 * Returned Value:
 *   On success, a pointer to the CXD56 lower half ADC driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

int cxd56_adcinitialize(void)
{
  int ret = OK;

#if defined (CONFIG_CXD56_LPADC0) || defined (CONFIG_CXD56_LPADC0_1) || defined (CONFIG_CXD56_LPADC_ALL)
  ret = register_driver("/dev/lpadc0", &g_adcops, 0666, &g_lpadc0priv);
  if (ret < 0)
    {
      aerr("Failed to register driver(lpadc0): %d\n", ret);
      return ret;
    }
#endif

#if defined (CONFIG_CXD56_LPADC1) || defined (CONFIG_CXD56_LPADC0_1) || defined (CONFIG_CXD56_LPADC_ALL)
  ret = register_driver("/dev/lpadc1", &g_adcops, 0666, &g_lpadc1priv);
  if (ret < 0)
    {
      aerr("Failed to register driver(lpadc1): %d\n", ret);
      return ret;
    }
#endif

#if defined (CONFIG_CXD56_LPADC2) || defined (CONFIG_CXD56_LPADC_ALL)
  ret = register_driver("/dev/lpadc2", &g_adcops, 0666, &g_lpadc2priv);
  if (ret < 0)
    {
      aerr("Failed to register driver(lpadc2): %d\n", ret);
      return ret;
    }
#endif

#if defined (CONFIG_CXD56_LPADC3) || defined (CONFIG_CXD56_LPADC_ALL)
  ret = register_driver("/dev/lpadc3", &g_adcops, 0666, &g_lpadc3priv);
  if (ret < 0)
    {
      aerr("Failed to register driver(lpadc3): %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_CXD56_HPADC0
  ret = register_driver("/dev/hpadc0", &g_adcops, 0666, &g_hpadc0priv);
  if (ret < 0)
    {
      aerr("Failed to register driver(hpadc0): %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_CXD56_HPADC1
  ret = register_driver("/dev/hpadc1", &g_adcops, 0666, &g_hpadc1priv);
  if (ret < 0)
    {
      aerr("Failed to register driver(hpadc1): %d\n", ret);
      return ret;
    }
#endif

  return ret;
}

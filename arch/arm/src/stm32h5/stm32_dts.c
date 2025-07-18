/****************************************************************************
 * arch/arm/src/stm32h5/stm32_dts.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sensors/sensor.h>
#include <nuttx/uorb.h>
#include <arch/board/board.h>

#include "stm32_rcc.h"
#include "hardware/stm32_dts.h"
#include "stm32_dts.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int      stm32_dts_activate   (struct sensor_lowerhalf_s *lower,
                                        struct file *filep, bool enable);
static int      stm32_dts_set_interval(struct sensor_lowerhalf_s *lower,
                                         struct file *filep,
                                         uint32_t *period_us);
static ssize_t  stm32_dts_fetch      (struct sensor_lowerhalf_s *lower,
                                        struct file *filep,
                                        char *buffer, size_t buflen);
static int      stm32_dts_get_info   (struct sensor_lowerhalf_s *lower,
                                        struct file *filep,
                                        struct sensor_device_info_s *info);
static int      stm32_dts_isr (int irq, void *context, void *arg);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_STM32H5_DTS_TRIGGER != 0
# error "Hardware triggers not implemented. Need LP Timers first."
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct dts_cal_s g_dts_cal =
{
  .fmt0    = 0,
  .ramp    = 0,
  .t0      = 0.0,
};

static struct dts_cfg_s g_dts_cfg =
{
  .samples        = 1,
  .lse            = false,
  .clk_frequency  = 0,
};

static struct sensor_lowerhalf_s g_dts_lower =
{
  .type    = SENSOR_TYPE_TEMPERATURE,
  .nbuffer = 1,
  .ops     = NULL,            /* bound in register() */
  .persist = false,
};

static const struct sensor_ops_s g_dts_ops =
{
  .open           = NULL,                     /* no per‑open activation */
  .close          = NULL,
  .activate       = stm32_dts_activate,
  .set_interval   = stm32_dts_set_interval,
  .batch          = NULL,
#if CONFIG_STM32H5_DTS_TRIGGER == 0
  .fetch          = stm32_dts_fetch,
#else
  .fetch          = NULL,
#endif
  .selftest       = NULL,
  .set_calibvalue = NULL,
  .calibrate      = NULL,
  .get_info       = stm32_dts_get_info,
  .control        = NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dts_activate
 *
 * Description:
 *   Enable or disable the DTS block and its clock.  Configure CFGR1 for
 *   sample time, reference clock, and trigger mode.  Wait for the initial
 *   calibration to complete (TS1_RDY = 1).
 ****************************************************************************/

static int stm32_dts_activate(struct sensor_lowerhalf_s *lower,
                              struct file *filep, bool enable)
{
  uint32_t t0valr1;
  uint8_t  t0valr1_t0;
  uint32_t regval  = 0;

  if (enable)
    {
      modifyreg32(STM32_RCC_APB1HENR, 0, RCC_APB1HENR_DTSEN);

      /* Compute PCLK prescaler ≤ 1MHz */

      uint32_t div = (STM32_PCLK1_FREQUENCY + 1000000 - 1) / 1000000;
      if (div > 127) div = 127;
      else if (div == 0) div = 1;

      /* Configure CFGR1: enable block, sample time, ref‑clk */

      uint32_t cfgr1 =
          DTS_CFGR1_TS1_EN
        | DTS_CFGR1_TS1_SMP_TIME(CONFIG_STM32H5_DTS_SMP_TIME)
#if !defined(CONFIG_STM32H5_DTS_REFCLK_LSE)
        | DTS_CFGR1_HSREF_CLK_DIV_RATIO(div)
#else
        | DTS_CFGR1_REFCLK_SEL
#endif
        | DTS_CFGR1_TS1_INTRIG(CONFIG_STM32H5_DTS_TRIGGER);

      putreg32(cfgr1, STM32_DTS_CFGR1);

      /* Wait for initial calibration (TS1_RDY=1) */

      while (!(getreg32(STM32_DTS_SR) & DTS_SR_TS1_RDY));

      t0valr1 = getreg32(STM32_DTS_T0VALR1);
      g_dts_cal.fmt0 = (t0valr1
            & DTS_T0VALR1_TS1_FMT0_MASK) >> DTS_T0VALR1_TS1_FMT0_SHIFT;
      g_dts_cal.ramp = (getreg32(STM32_DTS_RAMPVALR)
            & DTS_RAMPVALR_TS1_RAMP_COEFF_MASK)
            >> DTS_RAMPVALR_TS1_RAMP_COEFF_SHIFT;
      t0valr1_t0 = (t0valr1
            & DTS_T0VALR1_TS1_T0_MASK) >> DTS_T0VALR1_TS1_T0_SHIFT;

      g_dts_cal.t0 = (t0valr1_t0 == 0) ? 30.0f : 130.0f;

#if defined(CONFIG_STM32H5_DTS_REFCLK_LSE)
      g_dts_cfg.lse = true;
#else
      g_dts_cfg.lse = false;
#endif
      g_dts_cfg.samples = ((cfgr1 & DTS_CFGR1_TS1_SMP_TIME_MASK) >>
                        DTS_CFGR1_TS1_SMP_TIME_SHIFT);
      g_dts_cfg.samples = (g_dts_cfg.samples == 0) ? 1 : g_dts_cfg.samples;

      g_dts_cfg.clk_frequency = (g_dts_cfg.lse) ? STM32_LSE_FREQUENCY :
                                              STM32_PCLK1_FREQUENCY;

#ifdef CONFIG_STM32H5_DTS_ITEN_ITEF
      regval |= DTS_ITENR_ITEEN;
#endif
#ifdef CONFIG_STM32H5_DTS_ITEN_ITLF
      regval |= DTS_ITENR_ITLEN;
#endif
#ifdef CONFIG_STM32H5_DTS_ITEN_ITHF
      regval |= DTS_ITENR_ITHEN;
#endif

#ifdef CONFIG_STM32H5_DTS_AITEN_AITEF
      regval |= DTS_ITENR_AITEEN;
#endif
#ifdef CONFIG_STM32H5_DTS_AITEN_AITLF
      regval |= DTS_ITENR_AITLEN;
#endif
#ifdef CONFIG_STM32H5_DTS_AITEN_AITHF
      regval |= DTS_ITENR_AITHEN;
#endif
      putreg32(regval, STM32_DTS_ITENR);
      irq_attach(STM32_IRQ_DTS, stm32_dts_isr, lower);
      up_enable_irq(STM32_IRQ_DTS);
    }
  else
    {
      /* Disable block‑level IRQ, NVIC, clock */

      putreg32(0, STM32_DTS_ITENR);
      up_disable_irq(STM32_IRQ_DTS);
      modifyreg32(STM32_RCC_APB1HENR, RCC_APB1HENR_DTSEN, 0);
    }

  return OK;
}

#if CONFIG_STM32H5_DTS_TRIGGER == 0
/****************************************************************************
 * Name: stm32_dts_fetch
 *
 * Description:
 *   Perform one software‐triggered temperature measurement
 ****************************************************************************/

static ssize_t stm32_dts_fetch(struct sensor_lowerhalf_s *lower,
                               struct file *filep,
                               char *buffer, size_t buflen)
{
  struct sensor_temp report;
  uint32_t raw;
  float    fmeas;

  /* Wait for sensor to be ready before starting */

  while (!(getreg32(STM32_DTS_SR) & DTS_SR_TS1_RDY));

  /* Set START = 1 to initiate one measurement */

  modifyreg32(STM32_DTS_CFGR1, 0, DTS_CFGR1_TS1_START);

  /* Wait for TS1_RDY → 1: conversion complete */

  while (!(getreg32(STM32_DTS_SR) & DTS_SR_TS1_RDY));

  /* Set START = 0 to stop measurements */

  modifyreg32(STM32_DTS_CFGR1, DTS_CFGR1_TS1_START, 0);

  raw = getreg32(STM32_DTS_DR) & DTS_DR_TS1_MFREQ_MASK;

  /* Convert raw → frequency */

  if (g_dts_cfg.lse)
    {
      /* (LSE) mode: count FM pulses in (samp) LSE cycles */

      fmeas = (float)raw * (float)STM32_LSE_FREQUENCY
            / (float)(g_dts_cfg.samples);
    }
  else
    {
      /* (PCLK) mode: count PCLK ticks in (samp) FM(T) cycles */

      fmeas = (float)STM32_PCLK1_FREQUENCY * (float)(g_dts_cfg.samples)
            / (float)raw;
    }

  report.timestamp   = sensor_get_timestamp();
  report.temperature = g_dts_cal.t0 +
                     ((fmeas - ((float)g_dts_cal.fmt0 * 100.0f)) /
                     (float)g_dts_cal.ramp);

  if (buflen < sizeof(report))
    {
      return -EINVAL;
    }

  memcpy(buffer, &report, sizeof(report));
  return sizeof(report);
}
#endif

static void stm32_dts_handle_result(struct sensor_lowerhalf_s *lower)
{
  struct sensor_temp report;
  uint32_t raw   = getreg32(STM32_DTS_DR) & DTS_DR_TS1_MFREQ_MASK;
  float    fmeas;

  /* Convert raw ==> freq */

  if (g_dts_cfg.lse)
    {
      fmeas = (float)raw * STM32_LSE_FREQUENCY / (float)(g_dts_cfg.samples);
    }
  else
    {
      fmeas = (float)STM32_PCLK1_FREQUENCY * (float)(g_dts_cfg.samples)
            / (float)raw;
    }

  /* Build the report */

  report.timestamp   = sensor_get_timestamp();
  report.temperature = g_dts_cal.t0 +
                     ((fmeas - ((float)g_dts_cal.fmt0 * 100.0f)) /
                     (float)g_dts_cal.ramp);

  /* Push into NuttX sensor framework */

  lower->push_event(lower->priv, &report, sizeof(report));
}

/****************************************************************************
 * Name: stm32_dts_set_interval
 ****************************************************************************/

static int stm32_dts_set_interval(struct sensor_lowerhalf_s *lower,
                                    struct file *filep,
                                    uint32_t *period_us)
{
  return OK;
}

/****************************************************************************
 * Name: stm32_dts_get_info
 ****************************************************************************/

static int stm32_dts_get_info(struct sensor_lowerhalf_s *lower,
                                struct file *filep,
                                struct sensor_device_info_s *info)
{
  /* One factory‐calibrated internal sensor on the STM32H5:
   *
   *  Power:        ≈0.5 mW typical when measuring (≈150 uA@3.3 V)
   *  Max range:    –40 C…+125 C
   *  Resolution:   ≈0.15 C => approximation with 15 samples at 125 MHz
   *  Min delay:    ≈200 us per conversion (default sample time)
   *  Max delay:    0 (one‑shot, not batched)
   *  FIFO events   1 (no hardware FIFO)
   */

  DEBUGASSERT(info);
  info->version                   = 1;
  info->power                     = 0.5f;
  info->max_range                 = 165.0f;
  info->resolution                = 0.15f;
  info->min_delay                 = 200;
  info->max_delay                 = 0;
  info->fifo_reserved_event_count = 1;
  info->fifo_max_event_count      = 1;
  strlcpy(info->name,   "STM32H5 DTS", SENSOR_INFO_NAME_SIZE);
  strlcpy(info->vendor, "STMicro",     SENSOR_INFO_NAME_SIZE);
  return OK;
}

/****************************************************************************
 * Name: stm32_dts_isr
 *
 * Description:
 ****************************************************************************/

static int stm32_dts_isr(int irq, void *context, void *arg)
{
  struct sensor_lowerhalf_s *lower = (struct sensor_lowerhalf_s *)arg;
  uint32_t sr = getreg32(STM32_DTS_SR);
  bool pushed = false;

  /* End‑of‑measurement interrupt? */

  if (sr & DTS_SR_TS1_ITEF)
    {
      putreg32(DTS_ICIFR_CITEF, STM32_DTS_ICIFR);
      stm32_dts_handle_result(lower);
      pushed = true;
    }

  /* Low‑threshold crossed? */

  if (sr & DTS_SR_TS1_ITLF)
    {
      putreg32(DTS_ICIFR_CITLF, STM32_DTS_ICIFR);
      if (!pushed)
        {
          stm32_dts_handle_result(lower);
        }

      pushed = true;
    }

  /* High‑threshold crossed? */

  if (sr & DTS_SR_TS1_ITHF)
    {
      putreg32(DTS_ICIFR_CITHF, STM32_DTS_ICIFR);
      if (!pushed)
        {
          stm32_dts_handle_result(lower);
        }
    }

  lower->notify_event(lower->priv);

  return OK;
}

/****************************************************************************
 * Name: stm32_dts_register
 *
 ****************************************************************************/

int stm32h5_dts_register(int devno)
{
  int ret;

  g_dts_lower.ops = &g_dts_ops;

  /* Register the driver at /dev/uorb/sensor_tempN */

  ret = sensor_register(&g_dts_lower, devno);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

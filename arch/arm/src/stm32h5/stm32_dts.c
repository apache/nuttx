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

static float dts_get_fmt_hz          (uint16_t mfreq);

static float dts_get_temp_celsius    (float fmt_hz);

static void dts_handle_result        (struct sensor_lowerhalf_s *lower);

static void dts_configure_cfgr1      (void);

static void dts_get_cal_data         (void);

static void dts_get_cfg_data         (void);

static int stm32_dts_activate        (struct sensor_lowerhalf_s *lower,
                                      struct file *filep, bool enable);

static int stm32_dts_set_interval    (struct sensor_lowerhalf_s *lower,
                                      struct file *filep,
                                      uint32_t *period_us);

#if CONFIG_STM32H5_DTS_TRIGGER == 0
static ssize_t stm32_dts_fetch       (struct sensor_lowerhalf_s *lower,
                                      struct file *filep,
                                      char *buffer, size_t buflen);
#endif

static int stm32_dts_get_info        (struct sensor_lowerhalf_s *lower,
                                      struct file *filep,
                                      struct sensor_device_info_s *info);

static int stm32_dts_isr             (int irq, void *context, void *arg);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_STM32H5_DTS_TRIGGER != 0
# error "Hardware triggers not implemented. Need LP Timers first."
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32_dts_cal_s g_dts_cal =
{
  .fmt0    = 0.0f,
  .ramp    = 1.0f,
  .t0      = 30.0f,
};

static struct stm32_dts_cfg_s g_dts_cfg =
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
 * Name: dts_get_fmt_hz
 *
 * Description:
 *   Compute the value of FM(T) based on the REF_CLK and mfreq.
 *   See RM0481, Rev 4, section 27.3.7
 *
 * Input Parameters:
 *   mfreq - The value of TS1_MFREQ in DTS_DR.
 *
 * Returned Value:
 *   The value of FM(T) in Hz.
 *
 ****************************************************************************/

static float dts_get_fmt_hz(uint16_t mfreq)
{
  /* Convert mfreq -> fmt_hz
   * fmt_hz is proportional to clk_ptat based on temperature
   */

  if (g_dts_cfg.lse)
    {
      /* (LSE) mode: count FM pulses in (samp) LSE cycles */

      return ((float)mfreq * (float)STM32_LSE_FREQUENCY
             / (float)(g_dts_cfg.samples));
    }
  else
    {
      /* (PCLK) mode: count PCLK ticks in (samp) FM(T) cycles */

      return ((float)STM32_PCLK1_FREQUENCY * (float)(g_dts_cfg.samples)
             / (float)mfreq);
    }
}

/****************************************************************************
 * Name: dts_get_temp_celcius
 *
 * Description:
 *   Uses calibration data (T0, FM(T0), Ramp Coefficient) and the derived
 *   FM(T) to compute the temperature measured by the DTS in degrees
 *   celsius. See RM0481, Rev 4, section 27.3.7
 *
 * Input Parameters:
 *   fmt_hz - The value of FM(T) in Hz.
 *
 * Returned Value:
 *   The temperature read by the DTS in degrees celsius
 *
 ****************************************************************************/

static float dts_get_temp_celsius(float fmt_hz)
{
  /* Convert fmt_hz -> temperature in degrees Celsius */

  return g_dts_cal.t0 +
         ((fmt_hz - ((float)g_dts_cal.fmt0 * 100.0f)) /
         (float)g_dts_cal.ramp);
}

/****************************************************************************
 * Name: dts_handle_result
 *
 * Description:
 *   Put the computed temperature and timestamp in a sensor_temp structure
 *   and push the sensor event
 *
 * Input Parameters:
 *   *lower - Pointer to sensor lower_half driver
 *
 ****************************************************************************/

static void dts_handle_result(struct sensor_lowerhalf_s *lower)
{
  struct sensor_temp report;
  uint16_t mfreq   = getreg32(STM32_DTS_DR) & DTS_DR_TS1_MFREQ_MASK;
  float    fmt_hz  = dts_get_fmt_hz(mfreq);

  /* Build the report */

  report.timestamp   = sensor_get_timestamp();
  report.temperature = dts_get_temp_celsius(fmt_hz);

  /* Push into NuttX sensor framework */

  lower->push_event(lower->priv, &report, sizeof(report));
}

/****************************************************************************
 * Name: dts_configure_cfgr1
 *
 * Description:
 *   Write DTS_CFGR1 register based on Kconfig options and the REF_CLK.
 *
 ****************************************************************************/

static void dts_configure_cfgr1(void)
{
  /* Compute PCLK prescaler <= 1MHz */

#if !defined(CONFIG_STM32H5_DTS_REFCLK_LSE)
  uint32_t div = (STM32_PCLK1_FREQUENCY + 1000000 - 1) / 1000000;

  if (div > 127)
    {
      div = 127;
    }
  else if (div == 0)
    {
      div = 1;
    }
#endif

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
}

/****************************************************************************
 * Name: dts_get_cal_data
 *
 * Description:
 *   Obtain DTS Calibration data (T0, FM(T0), Ramp Coefficient).
 *   Store the calibration data in a global data structure. This data is
 *   constant and needs to be loaded only once.
 *
 ****************************************************************************/

static void dts_get_cal_data(void)
{
  uint32_t t0valr1 = getreg32(STM32_DTS_T0VALR1);

  g_dts_cal.fmt0 = (t0valr1
        & DTS_T0VALR1_TS1_FMT0_MASK) >> DTS_T0VALR1_TS1_FMT0_SHIFT;

  g_dts_cal.ramp = (getreg32(STM32_DTS_RAMPVALR)
        & DTS_RAMPVALR_TS1_RAMP_COEFF_MASK)
        >> DTS_RAMPVALR_TS1_RAMP_COEFF_SHIFT;

  uint8_t t0valr1_t0 = (t0valr1
        & DTS_T0VALR1_TS1_T0_MASK) >> DTS_T0VALR1_TS1_T0_SHIFT;

  g_dts_cal.t0 = (t0valr1_t0 == 0) ? 30.0f : 130.0f;
}

/****************************************************************************
 * Name: dts_get_cfg_data
 *
 * Description:
 *   Store select configuration data for later use.
 *
 ****************************************************************************/

static void dts_get_cfg_data(void)
{
  uint32_t cfgr1 = getreg32(STM32_DTS_CFGR1);

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
}

/****************************************************************************
 * Name: dts_configure_interrupts
 *
 * Description:
 *   Write the DTS_ITENR register based on Kconfig options.
 *
 * Input Parameters:
 *   *lower - Pointer to sensor lower_half driver
 *
 ****************************************************************************/

static void dts_configure_interrupts(struct sensor_lowerhalf_s *lower)
{
  uint32_t itenr = 0;

#ifdef CONFIG_STM32H5_DTS_ITEN_ITEF
  itenr |= DTS_ITENR_ITEEN;
#endif
#ifdef CONFIG_STM32H5_DTS_ITEN_ITLF
  itenr |= DTS_ITENR_ITLEN;
#endif
#ifdef CONFIG_STM32H5_DTS_ITEN_ITHF
  itenr |= DTS_ITENR_ITHEN;
#endif

#ifdef CONFIG_STM32H5_DTS_AITEN_AITEF
  itenr |= DTS_ITENR_AITEEN;
#endif
#ifdef CONFIG_STM32H5_DTS_AITEN_AITLF
  itenr |= DTS_ITENR_AITLEN;
#endif
#ifdef CONFIG_STM32H5_DTS_AITEN_AITHF
  itenr |= DTS_ITENR_AITHEN;
#endif

  putreg32(itenr, STM32_DTS_ITENR);
}

/****************************************************************************
 * Name: stm32_dts_activate
 *
 * Description:
 *   Enable or disable the DTS block and its clock.  Configure CFGR1 for
 *   sample time, reference clock, and trigger mode.  Wait for the initial
 *   calibration to complete (TS1_RDY = 1).
 *
 ****************************************************************************/

static int stm32_dts_activate(struct sensor_lowerhalf_s *lower,
                              struct file *filep, bool enable)
{
  if (enable)
    {
      modifyreg32(STM32_RCC_APB1HENR, 0, RCC_APB1HENR_DTSEN);

      dts_configure_cfgr1();

      /* Wait for initial calibration (TS1_RDY=1) */

      while (!(getreg32(STM32_DTS_SR) & DTS_SR_TS1_RDY));

      dts_get_cal_data();

      dts_get_cfg_data();

      dts_configure_interrupts(lower);

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
 *
 ****************************************************************************/

static ssize_t stm32_dts_fetch(struct sensor_lowerhalf_s *lower,
                               struct file *filep,
                               char *buffer, size_t buflen)
{
  struct sensor_temp report;
  uint16_t mfreq;
  float    fmt_hz;

  /* Wait for sensor to be ready before starting */

  while (!(getreg32(STM32_DTS_SR) & DTS_SR_TS1_RDY));

  /* Set START = 1 to initiate one measurement */

  modifyreg32(STM32_DTS_CFGR1, 0, DTS_CFGR1_TS1_START);

  /* Wait for TS1_RDY -> 1: conversion complete */

  while (!(getreg32(STM32_DTS_SR) & DTS_SR_TS1_RDY));

  /* Set START = 0 to stop measurements */

  modifyreg32(STM32_DTS_CFGR1, DTS_CFGR1_TS1_START, 0);

  mfreq = getreg32(STM32_DTS_DR) & DTS_DR_TS1_MFREQ_MASK;
  fmt_hz = dts_get_fmt_hz(mfreq);

  report.timestamp   = sensor_get_timestamp();
  report.temperature = dts_get_temp_celsius(fmt_hz);

  if (buflen < sizeof(report))
    {
      return -EINVAL;
    }

  memcpy(buffer, &report, sizeof(report));
  return sizeof(report);
}
#endif

/****************************************************************************
 * Name: stm32_dts_set_interval
 ****************************************************************************/

static int stm32_dts_set_interval(struct sensor_lowerhalf_s *lower,
                                    struct file *filep,
                                    uint32_t *period_us)
{
  /* TODO - Requires LP Timer Driver */

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
      dts_handle_result(lower);
      pushed = true;
    }

  /* Low‑threshold crossed? */

  if (sr & DTS_SR_TS1_ITLF)
    {
      putreg32(DTS_ICIFR_CITLF, STM32_DTS_ICIFR);
      if (!pushed)
        {
          dts_handle_result(lower);
        }

      pushed = true;
    }

  /* High‑threshold crossed? */

  if (sr & DTS_SR_TS1_ITHF)
    {
      putreg32(DTS_ICIFR_CITHF, STM32_DTS_ICIFR);
      if (!pushed)
        {
          dts_handle_result(lower);
        }
    }

  lower->notify_event(lower->priv);

  return OK;
}

/****************************************************************************
 * Name: stm32_dts_register
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

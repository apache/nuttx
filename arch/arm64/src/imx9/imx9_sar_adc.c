/****************************************************************************
 * arch/arm64/src/imx9/imx9_sar_adc.c
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

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include <debug.h>
#include <syslog.h>

#include "arm64_internal.h"

#include "imx9_sar_adc.h"
#include "imx9_ccm.h"
#include "hardware/imx9_ccm.h"
#include "hardware/imx93/imx9_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMX9_SAR_ADC_MAX_CHANNELS    4
#define IMX9_SAR_ADC_VALID_MASK      ((1u << IMX9_SAR_ADC_MAX_CHANNELS) - 1u)
#define IMX9_SAR_ADC_TIMEOUT_LOOPS   100000

#define IMX9_SAR_ADC_MCR_OFFSET      0x0000
#define IMX9_SAR_ADC_MSR_OFFSET      0x0004
#define IMX9_SAR_ADC_ISR_OFFSET      0x0010
#define IMX9_SAR_ADC_CEOCFR0_OFFSET  0x0014
#define IMX9_SAR_ADC_CTR0_OFFSET     0x0094
#define IMX9_SAR_ADC_NCMR0_OFFSET    0x00a4
#define IMX9_SAR_ADC_PCDR0_OFFSET    0x0100
#define IMX9_SAR_ADC_PCDR_STRIDE     0x0004

#define IMX9_SAR_ADC_MCR_OWREN       (1u << 31)
#define IMX9_SAR_ADC_MCR_MODE        (1u << 29)
#define IMX9_SAR_ADC_MCR_NSTART      (1u << 24)
#define IMX9_SAR_ADC_MCR_CALSTART    (1u << 14)
#define IMX9_SAR_ADC_MCR_ADCLKSE     (1u << 8)
#define IMX9_SAR_ADC_MCR_ABORTCHAIN  (1u << 7)
#define IMX9_SAR_ADC_MCR_ABORT       (1u << 6)
#define IMX9_SAR_ADC_MCR_ACKO        (1u << 5)
#define IMX9_SAR_ADC_MCR_PWDN        (1u << 0)
#define IMX9_SAR_ADC_MSR_CALBUSY     (1u << 29)
#define IMX9_SAR_ADC_MSR_NSTART      (1u << 24)

#define IMX9_SAR_ADC_MSR_ADCSTATUS_MASK   0x7u
#define IMX9_SAR_ADC_MSR_STATUS_IDLE      0x0u
#define IMX9_SAR_ADC_MSR_STATUS_POWERDOWN 0x1u

#define IMX9_SAR_ADC_ISR_EOC         (1u << 1)
#define IMX9_SAR_ADC_ISR_ECH         (1u << 0)

#define IMX9_SAR_ADC_CTR0_RESET      0x00000014u
#define IMX9_SAR_ADC_RESULT_MASK     0x0fffu

#define IMX9_SAR_ADC_REG(offset)     (IMX9_ADC1_BASE + (offset))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_imx9_sar_adc_initialized;
static uint32_t g_imx9_sar_adc_channel_mask;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_sar_adc_channel_valid
 *
 * Description:
 *   Check whether the requested channel number is implemented by this
 *   single-instance SAR ADC driver.
 *
 * Input Parameters:
 *   channel - ADC channel index to validate
 *
 * Returned Value:
 *   true if the channel is supported; false otherwise.
 ****************************************************************************/

static bool imx9_sar_adc_channel_valid(uint8_t channel)
{
  return channel < IMX9_SAR_ADC_MAX_CHANNELS;
}

/****************************************************************************
 * Name: imx9_sar_adc_wait_for_idle
 *
 * Description:
 *   Wait until the SAR ADC is idle and not busy with either calibration or
 *   a previously started conversion.
 *
 * Returned Value:
 *   Zero (OK) if the ADC reaches idle state; otherwise -ETIMEDOUT.
 ****************************************************************************/

static int imx9_sar_adc_wait_for_idle(void)
{
  unsigned int timeout = IMX9_SAR_ADC_TIMEOUT_LOOPS;

  while (timeout-- > 0)
    {
      uint32_t msr = getreg32(IMX9_SAR_ADC_REG(IMX9_SAR_ADC_MSR_OFFSET));
      uint32_t status = msr & IMX9_SAR_ADC_MSR_ADCSTATUS_MASK;

      if (((msr & IMX9_SAR_ADC_MSR_CALBUSY) == 0) &&
          ((msr & IMX9_SAR_ADC_MSR_NSTART) == 0) &&
          (status == IMX9_SAR_ADC_MSR_STATUS_IDLE))
        {
          return 0;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: imx9_sar_adc_power_up
 *
 * Description:
 *   Clear the power-down bit and wait until the ADC reports idle state.
 *
 * Returned Value:
 *   Zero (OK) if the ADC powers up successfully; otherwise -ETIMEDOUT.
 ****************************************************************************/

static int imx9_sar_adc_power_up(void)
{
  uint32_t mcr = getreg32(IMX9_SAR_ADC_REG(IMX9_SAR_ADC_MCR_OFFSET));
  unsigned int timeout = IMX9_SAR_ADC_TIMEOUT_LOOPS;

  mcr &= ~IMX9_SAR_ADC_MCR_PWDN;
  putreg32(mcr, IMX9_SAR_ADC_REG(IMX9_SAR_ADC_MCR_OFFSET));

  while (timeout-- > 0)
    {
      uint32_t msr = getreg32(IMX9_SAR_ADC_REG(IMX9_SAR_ADC_MSR_OFFSET));
      uint32_t status = msr & IMX9_SAR_ADC_MSR_ADCSTATUS_MASK;

      if (status == IMX9_SAR_ADC_MSR_STATUS_IDLE)
        {
          return 0;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: imx9_sar_adc_calibrate
 *
 * Description:
 *   Start a SAR ADC calibration cycle and wait for the hardware to return
 *   to idle.
 *
 * Returned Value:
 *   Zero (OK) on successful calibration; a negated errno on failure.
 ****************************************************************************/

static int imx9_sar_adc_calibrate(void)
{
  uint32_t mcr;
  int ret;

  ret = imx9_sar_adc_wait_for_idle();
  if (ret < 0)
    {
      return ret;
    }

  mcr = getreg32(IMX9_SAR_ADC_REG(IMX9_SAR_ADC_MCR_OFFSET));
  mcr |= IMX9_SAR_ADC_MCR_CALSTART;
  putreg32(mcr, IMX9_SAR_ADC_REG(IMX9_SAR_ADC_MCR_OFFSET));

  return imx9_sar_adc_wait_for_idle();
}

/****************************************************************************
 * Name: imx9_sar_adc_hw_init
 *
 * Description:
 *   Low-level hardware bring-up hook. Clock root/LPCG enable, pinmux, TRDC
 *   access and any calibration sequence belong here.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 ****************************************************************************/

static int imx9_sar_adc_hw_init(void)
{
  uint32_t mcr;
  int ret;

  /* Bring up the ADC clock tree before touching the block registers. */

  ret = imx9_ccm_configure_root_clock(CCM_CR_ADC, OSC_24M, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = imx9_ccm_root_clock_on(CCM_CR_ADC, true);
  if (ret < 0)
    {
      return ret;
    }

  ret = imx9_ccm_gate_on(CCM_LPCG_ADC1, true);
  if (ret < 0)
    {
      imx9_ccm_root_clock_on(CCM_CR_ADC, false);
      return ret;
    }

  ret = imx9_sar_adc_power_up();
  if (ret < 0)
    {
      imx9_ccm_gate_on(CCM_LPCG_ADC1, false);
      imx9_ccm_root_clock_on(CCM_CR_ADC, false);
      return ret;
    }

  mcr = getreg32(IMX9_SAR_ADC_REG(IMX9_SAR_ADC_MCR_OFFSET));
  mcr &= ~(IMX9_SAR_ADC_MCR_MODE |
           IMX9_SAR_ADC_MCR_ADCLKSE |
           IMX9_SAR_ADC_MCR_ABORTCHAIN |
           IMX9_SAR_ADC_MCR_ABORT |
           IMX9_SAR_ADC_MCR_ACKO);
  mcr |= IMX9_SAR_ADC_MCR_OWREN;
  putreg32(mcr, IMX9_SAR_ADC_REG(IMX9_SAR_ADC_MCR_OFFSET));

  /* Keep the reset timing value initially. */

  putreg32(IMX9_SAR_ADC_CTR0_RESET,
           IMX9_SAR_ADC_REG(IMX9_SAR_ADC_CTR0_OFFSET));

  ret = imx9_sar_adc_calibrate();
  if (ret < 0)
    {
      imx9_ccm_gate_on(CCM_LPCG_ADC1, false);
      imx9_ccm_root_clock_on(CCM_CR_ADC, false);
      return ret;
    }

  /* Clear any stale status before the first conversion. */

  putreg32(0xffffffffu, IMX9_SAR_ADC_REG(IMX9_SAR_ADC_ISR_OFFSET));
  putreg32(0xffffffffu, IMX9_SAR_ADC_REG(IMX9_SAR_ADC_CEOCFR0_OFFSET));

  return 0;
}

/****************************************************************************
 * Name: imx9_sar_adc_hw_deinit
 *
 * Description:
 *   Shut down the single supported SAR ADC instance and disable its clocks.
 ****************************************************************************/

static void imx9_sar_adc_hw_deinit(void)
{
  imx9_ccm_gate_on(CCM_LPCG_ADC1, false);
  imx9_ccm_root_clock_on(CCM_CR_ADC, false);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_sar_adc_init
 *
 * Description:
 *   Initialize the i.MX9 SAR ADC block and record the enabled channel mask.
 *
 * Input Parameters:
 *   channel_mask - Bitmask of ADC channels that callers may read
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 ****************************************************************************/

int imx9_sar_adc_init(uint32_t channel_mask)
{
  int ret;

  if ((channel_mask & ~IMX9_SAR_ADC_VALID_MASK) != 0)
    {
      aerr("ERROR: invalid channel mask 0x%08" PRIx32 "\n", channel_mask);
      return -EINVAL;
    }

  if (g_imx9_sar_adc_initialized)
    {
      g_imx9_sar_adc_channel_mask = channel_mask;
      return 0;
    }

  ret = imx9_sar_adc_hw_init();
  if (ret < 0)
    {
      aerr("ERROR: SAR ADC hw init failed: %d\n", ret);
      return ret;
    }

  g_imx9_sar_adc_channel_mask = channel_mask;
  g_imx9_sar_adc_initialized = true;
  return 0;
}

/****************************************************************************
 * Name: imx9_sar_adc_deinit
 *
 * Description:
 *   Shut down the i.MX9 SAR ADC block and clear the initialized state.
 ****************************************************************************/

void imx9_sar_adc_deinit(void)
{
  if (!g_imx9_sar_adc_initialized)
    {
      return;
    }

  imx9_sar_adc_hw_deinit();
  g_imx9_sar_adc_channel_mask = 0;
  g_imx9_sar_adc_initialized = false;
}

/****************************************************************************
 * Name: imx9_sar_adc_read_channel
 *
 * Description:
 *   Perform a one-shot conversion on the requested channel and return the
 *   raw result.
 *
 * Input Parameters:
 *   channel - ADC channel to convert
 *   value   - Location to receive the raw ADC result
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 ****************************************************************************/

int imx9_sar_adc_read_channel(uint8_t channel, uint32_t *value)
{
  uint32_t mcr;
  uint32_t reg_offset;
  unsigned int timeout;

  if (value == NULL)
    {
      return -EINVAL;
    }

  if (!g_imx9_sar_adc_initialized)
    {
      return -EIO;
    }

  if (!imx9_sar_adc_channel_valid(channel))
    {
      return -EINVAL;
    }

  if ((g_imx9_sar_adc_channel_mask & (1u << channel)) == 0)
    {
      return -ENODEV;
    }

  if (imx9_sar_adc_wait_for_idle() < 0)
    {
      return -ETIMEDOUT;
    }

  /* One-shot single-channel normal conversion on group 0. */

  putreg32(1u << channel, IMX9_SAR_ADC_REG(IMX9_SAR_ADC_NCMR0_OFFSET));

  /* Clear stale end-of-conversion flags before starting the new
   * conversion.
   */

  putreg32(IMX9_SAR_ADC_ISR_EOC | IMX9_SAR_ADC_ISR_ECH,
           IMX9_SAR_ADC_REG(IMX9_SAR_ADC_ISR_OFFSET));
  putreg32(1u << channel, IMX9_SAR_ADC_REG(IMX9_SAR_ADC_CEOCFR0_OFFSET));

  mcr = getreg32(IMX9_SAR_ADC_REG(IMX9_SAR_ADC_MCR_OFFSET));
  mcr &= ~IMX9_SAR_ADC_MCR_MODE;
  mcr |= IMX9_SAR_ADC_MCR_NSTART;
  putreg32(mcr, IMX9_SAR_ADC_REG(IMX9_SAR_ADC_MCR_OFFSET));

  timeout = IMX9_SAR_ADC_TIMEOUT_LOOPS;
  while (timeout-- > 0)
    {
      uint32_t isr = getreg32(IMX9_SAR_ADC_REG(IMX9_SAR_ADC_ISR_OFFSET));

      if ((isr & IMX9_SAR_ADC_ISR_EOC) != 0)
        {
          reg_offset = IMX9_SAR_ADC_PCDR0_OFFSET +
                       ((uint32_t)channel * IMX9_SAR_ADC_PCDR_STRIDE);
          *value = getreg32(IMX9_SAR_ADC_REG(reg_offset)) &
                   IMX9_SAR_ADC_RESULT_MASK;

          putreg32(IMX9_SAR_ADC_ISR_EOC | IMX9_SAR_ADC_ISR_ECH,
                   IMX9_SAR_ADC_REG(IMX9_SAR_ADC_ISR_OFFSET));
          putreg32(1u << channel,
                   IMX9_SAR_ADC_REG(IMX9_SAR_ADC_CEOCFR0_OFFSET));
          return 0;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: imx9_sar_adc_is_initialized
 *
 * Description:
 *   Report whether the single supported SAR ADC instance has been
 *   initialized.
 *
 * Returned Value:
 *   true if the SAR ADC is initialized; false otherwise.
 ****************************************************************************/

bool imx9_sar_adc_is_initialized(void)
{
  return g_imx9_sar_adc_initialized;
}

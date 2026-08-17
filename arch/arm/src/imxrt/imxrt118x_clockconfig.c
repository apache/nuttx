/****************************************************************************
 * arch/arm/src/imxrt/imxrt118x_clockconfig.c
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
#include <stdbool.h>
#include <stdint.h>

#include <arch/barriers.h>
#include <arch/board/board.h>
#include <nuttx/arch.h>

#include "arm_internal.h"
#include "hardware/imxrt_ccm.h"
#include "hardware/imxrt_memorymap.h"
#include "hardware/rt118x/imxrt118x_ele.h"
#include "hardware/rt118x/imxrt118x_osc.h"
#include "hardware/rt118x/imxrt118x_pll.h"
#include "hardware/rt118x/imxrt118x_pmu.h"
#include "imxrt_clockconfig.h"
#include "imxrt_periphclks.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMXRT118X_RC24M_FREQUENCY       24000000u
#define IMXRT118X_RC400M_FREQUENCY      400000000u
#define IMXRT118X_XTAL_FREQUENCY        BOARD_XTAL_FREQUENCY
#define IMXRT118X_SYSPLL1_FREQUENCY     1000000000u
#define IMXRT118X_SYSPLL2_FREQUENCY     528000000u
#define IMXRT118X_SYSPLL3_FREQUENCY     480000000u
#define IMXRT118X_ELE_TIMEOUT           1000000u
#define IMXRT118X_PLL_TIMEOUT           1000000u
#define IMXRT118X_SYSPLL1_DIV           41u
#define IMXRT118X_SYSPLL1_NUMERATOR     178956970u
#define IMXRT118X_SYSPLL1_DENOMINATOR   0x0fffffffu
#define IMXRT118X_SYSPLL3_PFD3_FRAC     18u
#define IMXRT118X_PLL_REG_DELAY_US      100u
#define IMXRT118X_PLL_HOLD_DELAY_US     225u

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t g_imxrt118x_clock_status
  __attribute__((section(".clockstatus")));
volatile int32_t g_imxrt118x_clock_error
  __attribute__((section(".clockstatus")));

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool imxrt_wait_mask(uintptr_t address, uint32_t mask, bool set)
{
  unsigned int timeout;

  for (timeout = IMXRT118X_ELE_TIMEOUT; timeout > 0; timeout--)
    {
      if (((getreg32(address) & mask) != 0) == set)
        {
          return true;
        }
    }

  return false;
}

static int imxrt_ele_command(const uint32_t *request,
                             unsigned int request_size,
                             uint32_t expected, uint32_t *response)
{
  uint32_t header;
  unsigned int i;
  unsigned int size;

  for (i = 0; i < request_size; i++)
    {
      if (!imxrt_wait_mask(IMXRT_S3MUA_TSR, 1u << i, true))
        {
          return -ETIMEDOUT;
        }

      putreg32(request[i], IMXRT_S3MUA_TR(i));
    }

  if (!imxrt_wait_mask(IMXRT_S3MUA_RSR, 1u, true))
    {
      return -ETIMEDOUT;
    }

  header = getreg32(IMXRT_S3MUA_RR(0));
  size = (header >> 8) & 0xffu;
  if (header != expected || size < 2 || size > 4)
    {
      return -EPROTO;
    }

  response[0] = header;
  for (i = 1; i < size; i++)
    {
      if (!imxrt_wait_mask(IMXRT_S3MUA_RSR, 1u << i, true))
        {
          return -ETIMEDOUT;
        }

      response[i] = getreg32(IMXRT_S3MUA_RR(i));
    }

  return response[1] == IMXRT_ELE_RESPONSE_SUCCESS ? OK : -EACCES;
}

static int imxrt_ele_release_trdc(uint32_t resource)
{
  uint32_t request[2];
  uint32_t response[4];

  request[0] = IMXRT_ELE_RELEASE_RDC;
  request[1] = (resource << 8) | IMXRT_ELE_CORE_CM33_ID;
  return imxrt_ele_command(request, 2, IMXRT_ELE_RELEASE_RDC_RESPONSE,
                           response);
}

static int imxrt_ele_prepare_clocks(void)
{
  uint32_t request[1];
  uint32_t response[4];
  int ret;

  request[0] = IMXRT_ELE_GET_FW_STATUS;
  ret = imxrt_ele_command(request, 1, IMXRT_ELE_GET_FW_STATUS_RESPONSE,
                          response);
  if (ret < 0)
    {
      return ret;
    }

  g_imxrt118x_clock_status |= IMXRT_CLOCK_STATUS_ELE_READY;

  ret = imxrt_ele_release_trdc(IMXRT_ELE_TRDC_AON_ID);
  if (ret < 0)
    {
      return ret;
    }

  g_imxrt118x_clock_status |= IMXRT_CLOCK_STATUS_TRDC_AON;

  /* MEGA must be transferred before WAKEUP because WAKEUP controls access
   * to the MEGA TRDC.
   */

  ret = imxrt_ele_release_trdc(IMXRT_ELE_TRDC_MEGA_ID);
  if (ret < 0)
    {
      return ret;
    }

  g_imxrt118x_clock_status |= IMXRT_CLOCK_STATUS_TRDC_MEGA;

  ret = imxrt_ele_release_trdc(IMXRT_ELE_TRDC_WAKEUP_ID);
  if (ret < 0)
    {
      return ret;
    }

  g_imxrt118x_clock_status |= IMXRT_CLOCK_STATUS_TRDC_WAKEUP;
  return OK;
}

static void imxrt_delay_cycles(unsigned int cycles)
{
  volatile unsigned int count;

  for (count = cycles; count > 0; count--)
    {
      __asm__ __volatile__("nop");
    }
}

static void imxrt_clock_failure(int error) noreturn_function;
static void imxrt_clock_failure(int error)
{
  g_imxrt118x_clock_error = error;

  /* A running kernel would use the 240 MHz board timing constants.  Do not
   * continue at RC24M with incorrect scheduler and peripheral timing.
   */

  for (; ; )
    {
      __asm__ __volatile__("nop");
    }
}

static int imxrt_syspll3_initialize(void)
{
  uint32_t reg;
  unsigned int timeout;

  reg = PHY_LDO_OUTPUT_TARGET(0x10) | PHY_LDO_ENABLE |
        PHY_LDO_CURRENT_LIMIT_ENABLE;
  putreg32(reg, IMXRT_PHY_LDO_CTRL0);
  imxrt_delay_cycles(64);
  modifyreg32(IMXRT_PHY_LDO_CTRL0, PHY_LDO_CURRENT_LIMIT_ENABLE, 0);
  g_imxrt118x_clock_status |= IMXRT_CLOCK_STATUS_PLL_LDO;

  reg = getreg32(IMXRT_ANADIG_PLL_SYS3_CTRL);
  if ((reg & PLL_SYS3_POWERUP) == 0)
    {
      putreg32(PLL_SYS3_REG_ENABLE | PLL_SYS3_GATE,
               IMXRT_ANADIG_PLL_SYS3_CTRL);
      imxrt_delay_cycles(1024);

      reg = PLL_SYS3_REG_ENABLE | PLL_SYS3_GATE | PLL_SYS3_POWERUP |
            PLL_SYS3_HOLD_RING_OFF;
      putreg32(reg, IMXRT_ANADIG_PLL_SYS3_CTRL);
      imxrt_delay_cycles(1024);

      reg &= ~PLL_SYS3_HOLD_RING_OFF;
      putreg32(reg, IMXRT_ANADIG_PLL_SYS3_CTRL);
    }

  for (timeout = IMXRT118X_PLL_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(IMXRT_ANADIG_PLL_SYS3_CTRL) & PLL_SYS3_STABLE) != 0)
        {
          modifyreg32(IMXRT_ANADIG_PLL_SYS3_CTRL, PLL_SYS3_GATE,
                      PLL_SYS3_ENABLE | PLL_SYS3_DIV2_ENABLE);
          g_imxrt118x_clock_status |= IMXRT_CLOCK_STATUS_SYS_PLL3;
          return OK;
        }
    }

  return -ETIMEDOUT;
}

#define SOURCES_COMMON(a, b) \
  {IMXRT_CLK_OSC_RC24M, IMXRT_CLK_OSC_RC400M, (a), (b)}

/* The mux input ordering is root-specific.  This table is derived from the
 * RT1186 clock mux table in the NXP device SDK, not from the RT117x.
 */

static const uint8_t g_clock_sources[IMXRT_CCM_ROOT_COUNT][4] =
{
  SOURCES_COMMON(IMXRT_CLK_ARM_PLL,          IMXRT_CLK_SYS_PLL3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3,         IMXRT_CLK_ARM_PLL),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL1,         IMXRT_CLK_SYS_PLL2_PFD1),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL2,         IMXRT_CLK_SYS_PLL3_PFD2),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL2,         IMXRT_CLK_SYS_PLL3_PFD1),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3,         IMXRT_CLK_SYS_PLL2_PFD1),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL1_DIV5),
  SOURCES_COMMON(IMXRT_CLK_OSC24M,           IMXRT_CLK_SYS_PLL3_DIV2),
  SOURCES_COMMON(IMXRT_CLK_OSC24M,           IMXRT_CLK_SYS_PLL3_DIV2),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL1_DIV5),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL1_DIV5),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_PFD0,    IMXRT_CLK_SYS_PLL2_PFD0),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_PFD2,    IMXRT_CLK_SYS_PLL2_PFD1),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL2,         IMXRT_CLK_SYS_PLL1),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3,         IMXRT_CLK_OSC24M),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3,         IMXRT_CLK_OSC24M),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3,         IMXRT_CLK_OSC24M),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_PFD1,    IMXRT_CLK_SYS_PLL2),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_PFD1,    IMXRT_CLK_SYS_PLL2),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_PFD1,    IMXRT_CLK_SYS_PLL2),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL2_PFD2,    IMXRT_CLK_SYS_PLL1_DIV5),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL2_PFD2,    IMXRT_CLK_SYS_PLL1_DIV5),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL1,         IMXRT_CLK_SYS_PLL2_PFD0),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3,         IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL1_DIV2,    IMXRT_CLK_SYS_PLL1_DIV5),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL1_DIV2,    IMXRT_CLK_SYS_PLL1_DIV5),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3,         IMXRT_CLK_SYS_PLL2_PFD3),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_PFD3,    IMXRT_CLK_SYS_PLL2_PFD1),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL1_DIV2,    IMXRT_CLK_SYS_PLL1_DIV5),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL1_DIV2,    IMXRT_CLK_SYS_PLL1_DIV5),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL1_DIV2,    IMXRT_CLK_SYS_PLL1_DIV5),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL1_DIV2,    IMXRT_CLK_SYS_PLL1_DIV5),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL1_DIV2,    IMXRT_CLK_SYS_PLL1_DIV5),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL1_DIV2,    IMXRT_CLK_SYS_PLL1_DIV5),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL1_DIV2,    IMXRT_CLK_SYS_PLL1_DIV5),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL1_DIV2,    IMXRT_CLK_SYS_PLL1_DIV5),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL1,         IMXRT_CLK_AUDIO_PLL),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL1,         IMXRT_CLK_AUDIO_PLL),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL1,         IMXRT_CLK_AUDIO_PLL),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3,         IMXRT_CLK_SYS_PLL3_PFD1),
  SOURCES_COMMON(IMXRT_CLK_OSC24M,           IMXRT_CLK_SYS_PLL3_DIV2),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL1_DIV5,    IMXRT_CLK_AUDIO_PLL),
  SOURCES_COMMON(IMXRT_CLK_AUDIO_PLL,        IMXRT_CLK_SYS_PLL3_PFD2),
  SOURCES_COMMON(IMXRT_CLK_AUDIO_PLL,        IMXRT_CLK_SYS_PLL3_PFD2),
  SOURCES_COMMON(IMXRT_CLK_AUDIO_PLL,        IMXRT_CLK_SYS_PLL3_PFD2),
  SOURCES_COMMON(IMXRT_CLK_AUDIO_PLL,        IMXRT_CLK_SYS_PLL3_PFD2),
  SOURCES_COMMON(IMXRT_CLK_AUDIO_PLL,        IMXRT_CLK_SYS_PLL3_PFD2),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3,         IMXRT_CLK_AUDIO_PLL),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_AUDIO_PLL),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL3_DIV2,    IMXRT_CLK_SYS_PLL1_DIV2),
  SOURCES_COMMON(IMXRT_CLK_SYS_PLL1_DIV5,    IMXRT_CLK_ARM_PLL)
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_clockroot_configure
 *
 * Description:
 *   Configure an RT118x clock root.  The hardware encodes divider N as N-1.
 *
 ****************************************************************************/

int imxrt_clockroot_configure(unsigned int root, unsigned int mux,
                              unsigned int divider, bool enable)
{
  uint32_t reg;

  if (root >= IMXRT_CCM_ROOT_COUNT || mux > 3 || divider == 0 ||
      divider > 256)
    {
      return -EINVAL;
    }

  reg = CCM_ROOT_DIV(divider) | CCM_ROOT_MUX(mux);
  if (!enable)
    {
      reg |= CCM_ROOT_OFF;
    }

  putreg32(reg, IMXRT_CCM_ROOT_CTRL(root));
  UP_MB();

  /* Complete the CM33 peripheral write before returning. */

  (void)getreg32(IMXRT_CCM_ROOT_CTRL(root));

  return OK;
}

static uint32_t imxrt_pfd_frequency(enum imxrt_pll_e pll,
                                    unsigned int pfd)
{
  uint32_t frac;
  uint32_t reg;
  uint32_t source;

  if (pfd >= PLL_PFD_COUNT)
    {
      return 0;
    }

  if (pll == IMXRT_PLL_SYS2)
    {
      reg = getreg32(IMXRT_ANADIG_PLL_SYS2_PFD);
      source = imxrt_pll_frequency(IMXRT_PLL_SYS2);
    }
  else if (pll == IMXRT_PLL_SYS3)
    {
      reg = getreg32(IMXRT_ANADIG_PLL_SYS3_PFD);
      source = imxrt_pll_frequency(IMXRT_PLL_SYS3);
    }
  else
    {
      return 0;
    }

  if ((reg & PLL_PFD_GATE(pfd)) != 0)
    {
      return 0;
    }

  frac = (reg & PLL_PFD_FRAC_MASK(pfd)) >> PLL_PFD_FRAC_SHIFT(pfd);
  if (source == 0 || frac < 12 || frac > 35)
    {
      return 0;
    }

  return (uint32_t)(((uint64_t)source * 18u) / frac);
}

/****************************************************************************
 * Name: imxrt_pll_frequency
 ****************************************************************************/

uint32_t imxrt_pll_frequency(enum imxrt_pll_e pll)
{
  uint32_t ctrl;
  uint32_t div;
  uint32_t denom;
  uint32_t numer;
  uint32_t postdiv;
  uint64_t frequency;

  switch (pll)
    {
      case IMXRT_PLL_ARM:
        ctrl = getreg32(IMXRT_ANADIG_PLL_ARM_CTRL);
        if ((ctrl & (PLL_ARM_POWERUP | PLL_ARM_ENABLE | PLL_ARM_STABLE)) !=
            (PLL_ARM_POWERUP | PLL_ARM_ENABLE | PLL_ARM_STABLE) ||
            (ctrl & PLL_ARM_GATE) != 0)
          {
            return 0;
          }

        if ((ctrl & PLL_ARM_BYPASS) != 0)
          {
            return IMXRT118X_XTAL_FREQUENCY;
          }

        div = (ctrl & PLL_ARM_DIV_MASK) >> PLL_ARM_DIV_SHIFT;
        postdiv = 1u << (((ctrl & PLL_ARM_POSTDIV_MASK) >>
                          PLL_ARM_POSTDIV_SHIFT) + 1u);
        return (IMXRT118X_XTAL_FREQUENCY / (2u * postdiv)) * div;

      case IMXRT_PLL_SYS1:
        ctrl = getreg32(IMXRT_ANADIG_PLL_SYS1_CTRL);
        return ((ctrl & (PLL_SYS1_ENABLE | PLL_SYS1_STABLE |
                         PLL_SYS1_GATE)) ==
                (PLL_SYS1_ENABLE | PLL_SYS1_STABLE)) ?
               IMXRT118X_SYSPLL1_FREQUENCY : 0;

      case IMXRT_PLL_SYS2:
        ctrl = getreg32(IMXRT_ANADIG_PLL_SYS2_CTRL);
        return ((ctrl & (PLL_SYS2_POWERUP | PLL_SYS2_ENABLE |
                         PLL_SYS2_STABLE | PLL_SYS2_GATE)) ==
                (PLL_SYS2_POWERUP | PLL_SYS2_ENABLE | PLL_SYS2_STABLE)) ?
               IMXRT118X_SYSPLL2_FREQUENCY : 0;

      case IMXRT_PLL_SYS3:
        ctrl = getreg32(IMXRT_ANADIG_PLL_SYS3_CTRL);
        return ((ctrl & (PLL_SYS3_POWERUP | PLL_SYS3_ENABLE |
                         PLL_SYS3_STABLE | PLL_SYS3_GATE)) ==
                (PLL_SYS3_POWERUP | PLL_SYS3_ENABLE | PLL_SYS3_STABLE)) ?
               IMXRT118X_SYSPLL3_FREQUENCY : 0;

      case IMXRT_PLL_AUDIO:
        ctrl = getreg32(IMXRT_PLL_CTRL(IMXRT_AUDIO_PLL_BASE));
        if ((ctrl & (PLL_CTRL_POWERUP | PLL_CTRL_ENABLE |
                     PLL_CTRL_BYPASS)) !=
            (PLL_CTRL_POWERUP | PLL_CTRL_ENABLE))
          {
            return 0;
          }

        div = (ctrl & PLL_CTRL_DIV_MASK) >> PLL_CTRL_DIV_SHIFT;
        postdiv = 1u << ((ctrl & PLL_CTRL_POSTDIV_MASK) >>
                         PLL_CTRL_POSTDIV_SHIFT);
        numer = getreg32(IMXRT_PLL_NUM(IMXRT_AUDIO_PLL_BASE)) &
                PLL_NUM_MASK;
        denom = getreg32(IMXRT_PLL_DENOM(IMXRT_AUDIO_PLL_BASE)) &
                PLL_DENOM_MASK;
        if (denom == 0)
          {
            return 0;
          }

        frequency = (uint64_t)IMXRT118X_XTAL_FREQUENCY *
                    ((uint64_t)div * denom + numer);
        return (uint32_t)(frequency / ((uint64_t)denom * postdiv));

      default:
        return 0;
    }
}

/****************************************************************************
 * Name: imxrt_clocksource_frequency
 ****************************************************************************/

uint32_t imxrt_clocksource_frequency(enum imxrt_clock_source_e source)
{
  switch (source)
    {
      case IMXRT_CLK_OSC_RC24M:
        return IMXRT118X_RC24M_FREQUENCY;
      case IMXRT_CLK_OSC_RC400M:
        return IMXRT118X_RC400M_FREQUENCY;
      case IMXRT_CLK_OSC24M:
        return IMXRT118X_XTAL_FREQUENCY;
      case IMXRT_CLK_ARM_PLL:
        return imxrt_pll_frequency(IMXRT_PLL_ARM);
      case IMXRT_CLK_SYS_PLL1:
        return imxrt_pll_frequency(IMXRT_PLL_SYS1);
      case IMXRT_CLK_SYS_PLL1_DIV2:
        return imxrt_pll_frequency(IMXRT_PLL_SYS1) / 2u;
      case IMXRT_CLK_SYS_PLL1_DIV5:
        return imxrt_pll_frequency(IMXRT_PLL_SYS1) / 5u;
      case IMXRT_CLK_SYS_PLL2:
        return imxrt_pll_frequency(IMXRT_PLL_SYS2);
      case IMXRT_CLK_SYS_PLL2_PFD0:
      case IMXRT_CLK_SYS_PLL2_PFD1:
      case IMXRT_CLK_SYS_PLL2_PFD2:
      case IMXRT_CLK_SYS_PLL2_PFD3:
        return imxrt_pfd_frequency(IMXRT_PLL_SYS2,
                                   source - IMXRT_CLK_SYS_PLL2_PFD0);
      case IMXRT_CLK_SYS_PLL3:
        return imxrt_pll_frequency(IMXRT_PLL_SYS3);
      case IMXRT_CLK_SYS_PLL3_DIV2:
        return imxrt_pll_frequency(IMXRT_PLL_SYS3) / 2u;
      case IMXRT_CLK_SYS_PLL3_PFD0:
      case IMXRT_CLK_SYS_PLL3_PFD1:
      case IMXRT_CLK_SYS_PLL3_PFD2:
      case IMXRT_CLK_SYS_PLL3_PFD3:
        return imxrt_pfd_frequency(IMXRT_PLL_SYS3,
                                   source - IMXRT_CLK_SYS_PLL3_PFD0);
      case IMXRT_CLK_AUDIO_PLL:
        return imxrt_pll_frequency(IMXRT_PLL_AUDIO);
      default:
        return 0;
    }
}

/****************************************************************************
 * Name: imxrt_clockroot_frequency
 *
 * Description:
 *   Return the effective frequency for any RT118x CCM root.
 ****************************************************************************/

int imxrt_clockroot_frequency(unsigned int root, uint32_t *frequency)
{
  uint32_t divider;
  uint32_t mux;
  uint32_t reg;
  uint32_t source;

  if (root >= IMXRT_CCM_ROOT_COUNT || frequency == NULL)
    {
      return -EINVAL;
    }

  reg = getreg32(IMXRT_CCM_ROOT_CTRL(root));
  if ((reg & CCM_ROOT_OFF) != 0)
    {
      return -ENODEV;
    }

  divider = ((reg & CCM_ROOT_DIV_MASK) >> CCM_ROOT_DIV_SHIFT) + 1;

  mux = (reg & CCM_ROOT_MUX_MASK) >> CCM_ROOT_MUX_SHIFT;
  source = imxrt_clocksource_frequency(g_clock_sources[root][mux]);
  if (source == 0)
    {
      return -ENODEV;
    }

  *frequency = source / divider;
  return OK;
}

/****************************************************************************
 * Name: imxrt_clockgate_configure
 *
 * Description:
 *   Enable or disable an RT118x LPCG.
 *
 ****************************************************************************/

void imxrt_clockgate_configure(unsigned int gate, bool enable)
{
  uint32_t reg;

  DEBUGASSERT(gate < IMXRT_CCM_LPCG_COUNT);

  reg = getreg32(IMXRT_CCM_LPCG_DIR(gate));
  if (enable)
    {
      reg |= CCM_LPCG_DIR_ON;
    }
  else
    {
      reg &= ~CCM_LPCG_DIR_ON;
    }

  putreg32(reg, IMXRT_CCM_LPCG_DIR(gate));
  UP_MB();

  /* Complete the CM33 peripheral write before returning. */

  (void)getreg32(IMXRT_CCM_LPCG_DIR(gate));
}

/****************************************************************************
 * Name: imxrt_periphclk_configure
 *
 * Description:
 *   Configure an RT118x peripheral clock through the common i.MX RT
 *   peripheral-clock interface.
 *
 ****************************************************************************/

void imxrt_periphclk_configure(unsigned int gate, unsigned int value)
{
  imxrt_clockgate_configure(gate, value != CCM_CG_OFF);
}

#ifdef CONFIG_IMXRT_NETC
/* SYS_PLL1 (Ethernet PLL) + NETC/MAC roots matching Zephyr FRDM RT1186. */

static int imxrt_syspll1_initialize(void)
{
  uintptr_t base = IMXRT_ETHERNET_PLL_BASE;
  unsigned int timeout;
  uint32_t ctrl;

  if (imxrt_pll_frequency(IMXRT_PLL_SYS1) != 0)
    {
      g_imxrt118x_clock_status |= IMXRT_CLOCK_STATUS_SYS_PLL1;
      return OK;
    }

  /* Bypass while programming the fractional bank. */

  putreg32(PLL_CTRL_BYPASS, IMXRT_PLL_CTRL_SET(base));
  modifyreg32(IMXRT_ANADIG_PLL_SYS1_CTRL, 0, PLL_SYS1_ENABLE);

  putreg32(IMXRT118X_SYSPLL1_NUMERATOR, IMXRT_PLL_NUM(base));
  putreg32(IMXRT118X_SYSPLL1_DENOMINATOR, IMXRT_PLL_DENOM(base));
  putreg32(PLL_CTRL_DIV_MASK, IMXRT_PLL_CTRL_CLR(base));
  putreg32(IMXRT118X_SYSPLL1_DIV & PLL_CTRL_DIV_MASK,
           IMXRT_PLL_CTRL_SET(base));
  putreg32(PLL_CTRL_REG_ENABLE, IMXRT_PLL_CTRL_SET(base));
  up_udelay(IMXRT118X_PLL_REG_DELAY_US);

  putreg32(PLL_CTRL_POWERUP | PLL_CTRL_HOLD_RING_OFF,
           IMXRT_PLL_CTRL_SET(base));
  up_udelay(IMXRT118X_PLL_HOLD_DELAY_US);
  putreg32(PLL_CTRL_HOLD_RING_OFF, IMXRT_PLL_CTRL_CLR(base));

  for (timeout = IMXRT118X_PLL_TIMEOUT; timeout > 0; timeout--)
    {
      ctrl = getreg32(IMXRT_ANADIG_PLL_SYS1_CTRL);
      if ((ctrl & PLL_SYS1_STABLE) != 0)
        {
          putreg32(PLL_CTRL_ENABLE, IMXRT_PLL_CTRL_SET(base));
          modifyreg32(IMXRT_ANADIG_PLL_SYS1_CTRL, PLL_SYS1_GATE,
                      PLL_SYS1_DIV2_ENABLE | PLL_SYS1_DIV5_ENABLE);
          putreg32(PLL_CTRL_BYPASS, IMXRT_PLL_CTRL_CLR(base));
          g_imxrt118x_clock_status |= IMXRT_CLOCK_STATUS_SYS_PLL1;
          return OK;
        }
    }

  return -ETIMEDOUT;
}

static int imxrt_syspll3_pfd3_initialize(void)
{
  uint32_t pfd;
  uint32_t stable;
  unsigned int timeout;
  unsigned int shift = PLL_PFD_FRAC_SHIFT(3);

  pfd = getreg32(IMXRT_ANADIG_PLL_SYS3_PFD);
  if (((pfd >> shift) & 0x3fu) == IMXRT118X_SYSPLL3_PFD3_FRAC &&
      (pfd & PLL_PFD_GATE(3)) == 0)
    {
      return OK;
    }

  stable = pfd & PLL_PFD_STABLE(3);
  pfd |= PLL_PFD_GATE(3);
  putreg32(pfd, IMXRT_ANADIG_PLL_SYS3_PFD);

  pfd &= ~PLL_PFD_FRAC_MASK(3);
  pfd |= PLL_PFD_FRAC(3, IMXRT118X_SYSPLL3_PFD3_FRAC);
  putreg32(pfd, IMXRT_ANADIG_PLL_SYS3_PFD);

  putreg32(getreg32(IMXRT_ANADIG_PLL_SYS3_UPDATE) ^ PLL_PFD_UPDATE(3),
           IMXRT_ANADIG_PLL_SYS3_UPDATE);

  pfd = getreg32(IMXRT_ANADIG_PLL_SYS3_PFD);
  pfd &= ~PLL_PFD_GATE(3);
  putreg32(pfd, IMXRT_ANADIG_PLL_SYS3_PFD);

  for (timeout = IMXRT118X_PLL_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(IMXRT_ANADIG_PLL_SYS3_PFD) & PLL_PFD_STABLE(3)) !=
          stable)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

int imxrt_netc_clocks_configure(void)
{
  int ret;

  if ((g_imxrt118x_clock_status & IMXRT_CLOCK_STATUS_NETC) != 0)
    {
      return OK;
    }

  ret = imxrt_syspll1_initialize();
  if (ret < 0)
    {
      return ret;
    }

  ret = imxrt_syspll3_pfd3_initialize();
  if (ret < 0)
    {
      return ret;
    }

  /* Zephyr SoC: NETC = SYS_PLL3_PFD3 / 2 (= 240 MHz). */

  imxrt_clockroot_configure(CCM_CR_NETC, 2, 2, true);

  /* Zephyr FRDM: MAC0 RGMII 1G needs 125 MHz (SYS_PLL1_DIV2 / 4).
   * MAC2 uses the same source with /4 for switch port 2.
   */

  imxrt_clockroot_configure(CCM_CR_MAC0, 2, 4, true);
  imxrt_clockroot_configure(CCM_CR_MAC2, 2, 4, true);
  imxrt_clockgate_configure(CCM_CCGR_NETC, true);

  g_imxrt118x_clock_status |= IMXRT_CLOCK_STATUS_NETC;
  return OK;
}
#endif /* CONFIG_IMXRT_NETC */

/****************************************************************************
 * Name: imxrt_clockconfig
 *
 * Description:
 *   Configure the initial RT1180 clock roots used by NuttX.
 *
 ****************************************************************************/

void imxrt_clockconfig(void)
{
  int ret;

  g_imxrt118x_clock_status = 0;
  g_imxrt118x_clock_error = OK;
  imxrt_clockroot_configure(CCM_CR_LPUART0102, 0, 1, true);
  imxrt_clockgate_configure(CCM_CCGR_LPUART1, true);

  ret = imxrt_ele_prepare_clocks();
  if (ret < 0)
    {
      imxrt_clock_failure(ret);
    }

  ret = imxrt_syspll3_initialize();
  if (ret < 0)
    {
      imxrt_clock_failure(ret);
    }

  /* SYS_PLL3 is a fixed 480 MHz source.  Keep LPUART at 24 MHz and run the
   * Cortex-M33 at 240 MHz, matching the NXP RT118x reference clock tree.
   */

  imxrt_clockroot_configure(CCM_CR_LPUART0102, 2, 10, true);
  imxrt_clockroot_configure(CCM_CR_M33, 2, 2, true);

  g_imxrt118x_clock_status |= IMXRT_CLOCK_STATUS_ROOTS_CONFIGURED;
}

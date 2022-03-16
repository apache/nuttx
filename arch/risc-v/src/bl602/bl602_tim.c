/****************************************************************************
 * arch/risc-v/src/bl602/bl602_tim.c
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

#include <stdint.h>
#include <stdbool.h>

#include <hardware/bl602_timer.h>
#include "bl602_tim.h"
#include "riscv_internal.h"

/****************************************************************************
 * Name: bl602_data_setbits
 ****************************************************************************/

static void bl602_wdt_access(void)
{
  modifyreg32(BL602_TIMER_WFAR, TIMER_WFAR_MASK, 0xbaba);
  modifyreg32(BL602_TIMER_WSAR, TIMER_WSAR_MASK, 0xeb10);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_timer_getcompvalue
 *
 * Description:
 *   Get the specified channel and match comparator value.
 *
 * Input Parameters:
 *   timer_ch  - TIMER channel type.
 *   cmp_no    - TIMER comparator ID type.
 *
 * Returned Value:
 *   Match compare register value
 *
 ****************************************************************************/

uint32_t bl602_timer_getcompvalue(uint8_t timer_ch, uint8_t cmp_no)
{
  return getreg32(BL602_TIMER_TMR2_0 + \
                  4 * (TIMER_MAX_MATCH * timer_ch + cmp_no));
}

/****************************************************************************
 * Name: bl602_timer_setcompvalue
 *
 * Description:
 *   TIMER set specified channel and comparator compare value
 *
 * Input Parameters:
 *   timer_ch - TIMER channel type.
 *   cmp_no   - TIMER comparator ID type.
 *   val     - TIMER match compare register value.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bl602_timer_setcompvalue(uint8_t timer_ch,
                              uint8_t cmp_no,
                              uint32_t val)
{
  putreg32(val,
           BL602_TIMER_TMR2_0 + 4 * (TIMER_MAX_MATCH * timer_ch + cmp_no));
}

/****************************************************************************
 * Name: bl602_timer_getcountervalue
 *
 * Description:
 *   TIMER get the specified channel count value.
 *
 * Input Parameters:
 *   timer_ch - TIMER channel type
 *
 * Returned Value:
 *   TIMER count register value
 *
 ****************************************************************************/

uint32_t bl602_timer_getcountervalue(uint32_t timer_ch)
{
  uint32_t count;
  uint32_t tcvwr_addr;

  /* TO avoid risk of reading, don't read TCVWR directly
   * request for read.  Why?  This register is read-only.
   */

  tcvwr_addr = BL602_TIMER_TCVWR2 + 4 * timer_ch;
  putreg32(1, tcvwr_addr);

  /* Need wait */

  count = getreg32(tcvwr_addr);
  count = getreg32(tcvwr_addr);
  count = getreg32(tcvwr_addr);

  return count;
}

/****************************************************************************
 * Name: bl602_timer_getmatchstatus
 *
 * Description:
 *   TIMER get specified channel and comparator match status
 *
 * Input Parameters:
 *   timer_ch - TIMER channel type.
 *   cmp_no   - TIMER comparator ID type.
 *
 * Returned Value:
 *   match status
 *
 ****************************************************************************/

bool bl602_timer_getmatchstatus(uint32_t timer_ch, uint8_t cmp_no)
{
  uint32_t status = getreg32(BL602_TIMER_TMSR2 + 4 * timer_ch);

  switch (cmp_no)
    {
    case TIMER_COMP_ID_0:
      return ((status & TIMER_TMSR2_TMSR_0) != 0);
    case TIMER_COMP_ID_1:
      return ((status & TIMER_TMSR2_TMSR_1) != 0);
    case TIMER_COMP_ID_2:
      return ((status & TIMER_TMSR2_TMSR_2) != 0);
    default:
      break;
    }

  return false;
}

/****************************************************************************
 * Name: bl602_timer_getpreloadvalue
 *
 * Description:
 *   TIMER get specified channel preload value
 *
 * Input Parameters:
 *   timer_ch - TIMER channel type.
 *
 * Returned Value:
 *   Preload register value.
 *
 ****************************************************************************/

uint32_t bl602_timer_getpreloadvalue(uint32_t timer_ch)
{
  return getreg32(BL602_TIMER_TPLVR2 + 4 * timer_ch);
}

/****************************************************************************
 * Name: bl602_timer_setpreloadvalue
 *
 * Description:
 *   TIMER set preload register low 32bits value
 *
 * Input Parameters:
 *   timer_ch - TIMER channel type.
 *   val     - Preload register low 32bits value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_timer_setpreloadvalue(uint8_t timer_ch, uint32_t val)
{
    putreg32(val, BL602_TIMER_TPLVR2 + 4 * timer_ch);
}

/****************************************************************************
 * Name: bl602_timer_setpreloadtrigsrc
 *
 * Description:
 *   TIMER set preload trigger source,COMP0,COMP1,COMP2 or None
 *
 * Input Parameters:
 *   timer_ch - TIMER channel type.
 *   pl_src   - TIMER preload source type.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_timer_setpreloadtrigsrc(uint8_t timer_ch, uint32_t pl_src)
{
  putreg32(pl_src, BL602_TIMER_TPLCR2 + 4 * timer_ch);
}

/****************************************************************************
 * Name: bl602_timer_setcountmode
 *
 * Description:
 *   TIMER set count mode:preload or free run
 *
 * Input Parameters:
 *   timer_ch   - TIMER channel type.
 *   count_mode - TIMER count mode: TIMER_COUNT_PRELOAD or
 *                TIMER_COUNT_FREERUN.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_timer_setcountmode(uint32_t timer_ch, uint8_t count_mode)
{
  modifyreg32(BL602_TIMER_TCMR,
              TIMER_TCMR_TIMER2_MODE << timer_ch,
              count_mode ? TIMER_TCMR_TIMER2_MODE << timer_ch : 0);
}

/****************************************************************************
 * Name: bl602_timer_clearintstatus
 *
 * Description:
 *   TIMER clear interrupt status
 *
 * Input Parameters:
 *   timer_ch - TIMER channel type.
 *   cmp_no   - TIMER macth comparator ID type.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_timer_clearintstatus(uint8_t timer_ch, uint32_t cmp_no)
{
  putreg32(BL602_TIMER_TICR2 + 4 * timer_ch, 1 << cmp_no);
}

/****************************************************************************
 * Name: bl602_timer_init
 *
 * Description:
 *   TIMER initialization function.
 *
 * Input Parameters:
 *   timer_cfg - TIMER configuration structure pointer.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bl602_timer_init(struct timer_cfg_s *timer_cfg)
{
  uint8_t timer_ch = timer_cfg->timer_ch;

  /* Configure timer clock source */

  if (timer_ch == TIMER_CH0)
    {
      modifyreg32(BL602_TIMER_TCCR, TIMER_TCCR_CS_1_MASK,
                  timer_cfg->clk_src << TIMER_TCCR_CS_1_SHIFT);
    }
  else
    {
      modifyreg32(BL602_TIMER_TCCR, TIMER_TCCR_CS_2_MASK,
                  timer_cfg->clk_src << TIMER_TCCR_CS_2_SHIFT);
    }

  /* Configure timer clock division */

  if (timer_ch == TIMER_CH0)
    {
      modifyreg32(BL602_TIMER_TCDR, TIMER_TCDR_TCDR2_MASK,
                  timer_cfg->clock_division << TIMER_TCDR_TCDR2_SHIFT);
    }
  else
    {
      modifyreg32(BL602_TIMER_TCDR, TIMER_TCDR_TCDR3_MASK,
                  timer_cfg->clock_division << TIMER_TCDR_TCDR3_SHIFT);
    }

  /* Configure timer count mode: preload or free run */

  bl602_timer_setcountmode(timer_ch, timer_cfg->count_mode);

  /* Configure timer preload trigger src */

  bl602_timer_setpreloadtrigsrc(timer_ch, timer_cfg->pl_trig_src);

  if (timer_cfg->count_mode == TIMER_COUNT_PRELOAD)
    {
      /* Configure timer preload value */

      bl602_timer_setpreloadvalue(timer_ch, timer_cfg->pre_load_val);
    }

  /* Configure match compare values */

  bl602_timer_setcompvalue(timer_ch, 0, timer_cfg->match_val0);
  bl602_timer_setcompvalue(timer_ch, 1, timer_cfg->match_val1);
  bl602_timer_setcompvalue(timer_ch, 2, timer_cfg->match_val2);
}

/****************************************************************************
 * Name: bl602_timer_enable
 *
 * Description:
 *   TIMER enable one channel function.
 *
 * Input Parameters:
 *   timer_ch - TIMER channel type.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_timer_enable(uint8_t timer_ch)
{
  modifyreg32(BL602_TIMER_TCER, 0, (TIMER_TCER_TIMER2_EN << timer_ch));
}

/****************************************************************************
 * Name: bl602_timer_disable
 *
 * Description:
 *   TIMER disable one channel function.
 *
 * Input Parameters:
 *   timer_ch - TIMER channel type.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_timer_disable(uint8_t timer_ch)
{
  modifyreg32(BL602_TIMER_TCER, (TIMER_TCER_TIMER2_EN << timer_ch), 0);
}

/****************************************************************************
 * Name: bl602_timer_intmask
 *
 * Description:
 *   TIMER mask or unmask certain or all interrupt.
 *
 * Input Parameters:
 *   timer_ch - TIMER channel type.
 *   int_type - TIMER interrupt type.
 *   int_mask - TIMER interrupt mask value:1:disable interrupt.0:enable
 *              interrupt.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_timer_intmask(uint8_t timer_ch, uint8_t int_type,
                         uint8_t int_mask)
{
  uint32_t tier_addr = BL602_TIMER_TIER2 + 4 * timer_ch;

  switch (int_type)
    {
    case TIMER_INT_COMP_0:
      if (int_mask == 0)
        {
          /* Enable this interrupt */

          modifyreg32(tier_addr, 0, TIMER_TIER2_TIER_0);
        }
      else
        {
          /* Disable this interrupt */

          modifyreg32(tier_addr, TIMER_TIER2_TIER_0, 0);
        }

      break;

    case TIMER_INT_COMP_1:
      if (int_mask == 0)
        {
          /* Enable this interrupt */

          modifyreg32(tier_addr, 0, TIMER_TIER2_TIER_1);
        }
      else
        {
          /* Disable this interrupt */

          modifyreg32(tier_addr, TIMER_TIER2_TIER_1, 0);
        }

      break;

    case TIMER_INT_COMP_2:
      if (int_mask == 0)
        {
          /* Enable this interrupt */

          modifyreg32(tier_addr, 0, TIMER_TIER2_TIER_2);
        }
      else
        {
          /* Disable this interrupt */

          modifyreg32(tier_addr, TIMER_TIER2_TIER_2, 0);
        }

      break;

    case TIMER_INT_ALL:
      if (int_mask == 0)
        {
          /* Enable this interrupt */

          modifyreg32(tier_addr, 0,
            TIMER_TIER2_TIER_0 | TIMER_TIER2_TIER_1 | TIMER_TIER2_TIER_2);
        }
      else
        {
          /* Disable this interrupt */

          modifyreg32(tier_addr,
            TIMER_TIER2_TIER_0 | TIMER_TIER2_TIER_1 | TIMER_TIER2_TIER_2, 0);
        }

      break;

    default:
      break;
    }
}

/****************************************************************************
 * Name: bl602_wdt_set_clock
 *
 * Description:
 *   TIMER set watchdog clock source and clock division.
 *
 * Input Parameters:
 *   clk_src - Watchdog timer clock source type.
 *   div - Watchdog timer clock division value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_wdt_set_clock(uint8_t clk_src, uint8_t div)
{
  /* Configure watchdog timer clock source */

  modifyreg32(BL602_TIMER_TCCR, TIMER_TCCR_CS_WDT_MASK,
              clk_src << TIMER_TCCR_CS_WDT_SHIFT);

  /* Configure watchdog timer clock division */

  modifyreg32(BL602_TIMER_TCDR, TIMER_TCDR_WCDR_MASK,
              div << TIMER_TCDR_WCDR_SHIFT);
}

/****************************************************************************
 * Name: bl602_wdt_getmatchvalue
 *
 * Description:
 *   TIMER get watchdog match compare value.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Watchdog match compare register value.
 *
 ****************************************************************************/

uint16_t bl602_wdt_getmatchvalue(void)
{
  bl602_wdt_access();

  /* Get watchdog timer match register value */

  return (uint16_t)(getreg32(BL602_TIMER_WMR) & TIMER_WMR_WMR_MASK);
}

/****************************************************************************
 * Name: bl602_wdt_setcompvalue
 *
 * Description:
 *   TIMER set watchdog match compare value.
 *
 * Input Parameters:
 *   val - Watchdog match compare value
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_wdt_setcompvalue(uint16_t val)
{
  bl602_wdt_access();

  /* Set watchdog timer match register value */

  putreg32(val, BL602_TIMER_WMR);
}

/****************************************************************************
 * Name: bl602_wdt_getcountervalue
 *
 * Description:
 *   TIMER get watchdog count register value.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Watchdog count register value.
 *
 ****************************************************************************/

uint16_t bl602_wdt_getcountervalue(void)
{
  bl602_wdt_access();

  /* Get watchdog timer count register value */

  return (uint16_t)(getreg32(BL602_TIMER_WVR) & TIMER_WVR_WVR_MASK);
}

/****************************************************************************
 * Name: bl602_wdt_resetcountervalue
 *
 * Description:
 *   TIMER reset watchdog count register value.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_wdt_resetcountervalue(void)
{
  /* Reset watchdog timer count register value */

  bl602_wdt_access();

  /* Set watchdog counter reset register bit0 to 1 */

  modifyreg32(BL602_TIMER_WCR, 0, TIMER_WCR_WCR);
}

/****************************************************************************
 * Name: bl602_wdt_getresetstatus
 *
 * Description:
 *   TIMER get watchdog reset status.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   0 or 1.
 *
 ****************************************************************************/

bool bl602_wdt_getresetstatus(void)
{
  bl602_wdt_access();

  /* Get watchdog status register */

  return ((getreg32(BL602_TIMER_WSR) & TIMER_WSR_WTS) != 0);
}

/****************************************************************************
 * Name: bl602_wdt_clearresetstatus
 *
 * Description:
 *   TIMER clear watchdog reset status.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_wdt_clearresetstatus(void)
{
  bl602_wdt_access();

  /* Set watchdog status register */

  modifyreg32(BL602_TIMER_WSR, TIMER_WSR_WTS, 0);
}

/****************************************************************************
 * Name: bl602_wdt_enable
 *
 * Description:
 *   TIMER enable watchdog function.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_wdt_enable(void)
{
  bl602_wdt_access();

  modifyreg32(BL602_TIMER_WMER, 0, TIMER_WMER_WE);
}

/****************************************************************************
 * Name: bl602_wdt_disable
 *
 * Description:
 *   Watchdog timer disable function.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_wdt_disable(void)
{
  bl602_wdt_access();

  modifyreg32(BL602_TIMER_WMER, TIMER_WMER_WE, 0);
}

/****************************************************************************
 * Name: bl602_wdt_intmask
 *
 * Description:
 *   Watchdog timer mask or unmask certain or all interrupt.
 *
 * Input Parameters:
 *   int_type - Watchdog interrupt type.
 *   int_mask - Watchdog interrupt mask value:BL_STD_MASK:disable
 *              interrupt.BL_STD_UNMASK:enable interrupt.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_wdt_intmask(uint8_t int_type, uint8_t int_mask)
{
  bl602_wdt_access();

  /* Deal with watchdog match/interrupt enable register,WRIE:watchdog
   * reset/interrupt enable
   */

  switch (int_type)
    {
    case WDT_INT:
      if (int_mask == 0)
        {
          /* Enable this interrupt */

          /* 0 means generates a watchdog interrupt,a watchdog timer reset is
           * not generated
           */

          modifyreg32(BL602_TIMER_WMER, TIMER_WMER_WRIE, 0);
        }
      else
        {
          /* Disable this interrupt */

          /* 1 means generates a watchdog timer reset,a watchdog interrupt is
           * not generated
           */

          modifyreg32(BL602_TIMER_WMER, 0, TIMER_WMER_WRIE);
        }

      break;
    default:
      break;
    }
}

/****************************************************************************
 * boards/risc-v/bl602/evb/src/bl602_tim.c
 *
 * Copyright (C) 2012, 2015 Gregory Nutt. All rights reserved.
 * Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <hardware/bl602_timer.h>
#include "riscv_arch.h"
#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TIMER_MAX_MATCH 3

/****************************************************************************
 * Static Functions
 ****************************************************************************/

static inline uint32_t bl602_up_tim_regin(uint32_t reg_addr)
{
  return getreg32(reg_addr);
}

static inline void bl602_up_tim_regout(uint32_t reg_addr, uint32_t value)
{
  putreg32(value, reg_addr);
}

/****************************************************************************
 * Name: bl602_data_setbits
 ****************************************************************************/

static uint32_t bl602_data_setbits(uint32_t data,
                                   uint32_t start,
                                   uint32_t len,
                                   uint32_t value)
{
  return (((data) & ~((~((~0) << (len))) << (start))) |
          (((value) & ((~((~0) << (len))))) << (start)));
}

static void bl602_wdt_access(void)
{
  uint32_t tmp_val = bl602_up_tim_regin(TIMER_BASE + TIMER_WFAR_OFFSET);

  bl602_up_tim_regout(
    TIMER_BASE + TIMER_WFAR_OFFSET,
    bl602_data_setbits(tmp_val, TIMER_WFAR_POS, 16, 0xbaba));

  tmp_val = bl602_up_tim_regin(TIMER_BASE + TIMER_WSAR_OFFSET);
  bl602_up_tim_regout(
    TIMER_BASE + TIMER_WSAR_OFFSET,
    bl602_data_setbits(tmp_val, TIMER_WSAR_POS, 16, 0xeb10));
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
 *   Match comapre register value
 *
 ****************************************************************************/

uint32_t bl602_timer_getcompvalue(uint32_t timer_ch, uint32_t cmp_no)
{
  uint32_t tmp_val;

  tmp_val = bl602_up_tim_regin(TIMER_BASE + TIMER_TMR2_0_OFFSET +
                               4 * (TIMER_MAX_MATCH * timer_ch + cmp_no));
  return tmp_val;
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
 *   val     - TIMER match comapre register value.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bl602_timer_setcompvalue(uint32_t timer_ch,
                              uint32_t cmp_no,
                              uint32_t val)
{
  bl602_up_tim_regout(TIMER_BASE + TIMER_TMR2_0_OFFSET +
                        4 * (TIMER_MAX_MATCH * timer_ch + cmp_no),
                      val);
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
  uint32_t tmp_val;
  uint32_t tmp_addr;

  /* TO avoid risk of reading, don't read TCVWR directly
   * request for read
   */

  tmp_addr = TIMER_BASE + TIMER_TCVWR2_OFFSET + 4 * timer_ch;
  bl602_up_tim_regout(tmp_addr, 1);

  /* Need wait */

  tmp_val = bl602_up_tim_regin(tmp_addr);
  tmp_val = bl602_up_tim_regin(tmp_addr);
  tmp_val = bl602_up_tim_regin(tmp_addr);

  return tmp_val;
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
 *   0 or 1
 *
 ****************************************************************************/

uint32_t bl602_timer_getmatchstatus(uint32_t timer_ch, uint32_t cmp_no)
{
  uint32_t tmp_val;
  uint32_t bit_status = 0;

  tmp_val =
    bl602_up_tim_regin(TIMER_BASE + TIMER_TMSR2_OFFSET + 4 * timer_ch);
  switch (cmp_no)
    {
    case TIMER_COMP_ID_0:
      bit_status = (((tmp_val) & (1 << (TIMER_TMSR_0_POS))) ? 1 : 0);
      break;
    case TIMER_COMP_ID_1:
      bit_status = (((tmp_val) & (1 << (TIMER_TMSR_1_POS))) ? 1 : 0);
      break;
    case TIMER_COMP_ID_2:
      bit_status = (((tmp_val) & (1 << (TIMER_TMSR_2_POS))) ? 1 : 0);
      break;
    default:
      break;
    }

  return bit_status;
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
  uint32_t tmp_val;
  tmp_val =
    bl602_up_tim_regin(TIMER_BASE + TIMER_TPLVR2_OFFSET + 4 * timer_ch);

  return tmp_val;
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

void bl602_timer_setpreloadvalue(uint32_t timer_ch, uint32_t val)
{
  bl602_up_tim_regout(TIMER_BASE + TIMER_TPLVR2_OFFSET + 4 * timer_ch, val);
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

void bl602_timer_setpreloadtrigsrc(uint32_t timer_ch, uint32_t pl_src)
{
  bl602_up_tim_regout(TIMER_BASE + TIMER_TPLCR2_OFFSET + 4 * timer_ch,
                      pl_src);
}

/****************************************************************************
 * Name: bl602_timer_setcountmode
 *
 * Description:
 *   TIMER set count mode:preload or free run
 *
 * Input Parameters:
 *   timer_ch - TIMER channel type.
 *   count_mode - TIMER count mode: TIMER_COUNT_PRELOAD or
 *TIMER_COUNT_FREERUN.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_timer_setcountmode(uint32_t timer_ch, uint32_t count_mode)
{
  uint32_t tmpval;

  tmpval = bl602_up_tim_regin(TIMER_BASE + TIMER_TCMR_OFFSET);
  tmpval &= (~(1 << (timer_ch + 1)));
  tmpval |= (count_mode << (timer_ch + 1));

  bl602_up_tim_regout(TIMER_BASE + TIMER_TCMR_OFFSET, tmpval);
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

void bl602_timer_clearintstatus(uint32_t timer_ch, uint32_t cmp_no)
{
  uint32_t tmp_addr;
  uint32_t tmp_val;

  tmp_addr = TIMER_BASE + TIMER_TICR2_OFFSET + 4 * timer_ch;

  tmp_val = bl602_up_tim_regin(tmp_addr);
  tmp_val |= (1 << cmp_no);

  bl602_up_tim_regout(tmp_addr, tmp_val);
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

void bl602_timer_init(timer_cfg_t *timer_cfg)
{
  uint32_t timer_ch = timer_cfg->timer_ch;
  uint32_t tmp_val;

  /* Configure timer clock source */

  tmp_val = bl602_up_tim_regin(TIMER_BASE + TIMER_TCCR_OFFSET);
  if (timer_ch == TIMER_CH0)
    {
      tmp_val =
        bl602_data_setbits(tmp_val, TIMER_CS_1_POS, 2, timer_cfg->clk_src);
    }
  else
    {
      tmp_val =
        bl602_data_setbits(tmp_val, TIMER_CS_2_POS, 2, timer_cfg->clk_src);
    }

  bl602_up_tim_regout(TIMER_BASE + TIMER_TCCR_OFFSET, tmp_val);

  /* Configure timer clock division */

  tmp_val = bl602_up_tim_regin(TIMER_BASE + TIMER_TCDR_OFFSET);
  if (timer_ch == TIMER_CH0)
    {
      tmp_val = bl602_data_setbits(
        tmp_val, TIMER_TCDR2_POS, 8, timer_cfg->clock_division);
    }
  else
    {
      tmp_val = bl602_data_setbits(
        tmp_val, TIMER_TCDR3_POS, 8, timer_cfg->clock_division);
    }

  bl602_up_tim_regout(TIMER_BASE + TIMER_TCDR_OFFSET, tmp_val);

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

  bl602_timer_setcompvalue(timer_ch, TIMER_COMP_ID_0, timer_cfg->match_val0);
  bl602_timer_setcompvalue(timer_ch, TIMER_COMP_ID_1, timer_cfg->match_val1);
  bl602_timer_setcompvalue(timer_ch, TIMER_COMP_ID_2, timer_cfg->match_val2);
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

void bl602_timer_enable(uint32_t timer_ch)
{
  uint32_t tmp_val;

  tmp_val = bl602_up_tim_regin(TIMER_BASE + TIMER_TCER_OFFSET);
  tmp_val |= (1 << (timer_ch + 1));

  bl602_up_tim_regout(TIMER_BASE + TIMER_TCER_OFFSET, tmp_val);
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

void bl602_timer_disable(uint32_t timer_ch)
{
  uint32_t tmp_val;

  tmp_val = bl602_up_tim_regin(TIMER_BASE + TIMER_TCER_OFFSET);
  tmp_val &= (~(1 << (timer_ch + 1)));

  bl602_up_tim_regout(TIMER_BASE + TIMER_TCER_OFFSET, tmp_val);
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
 *   int_mask - TIMER interrupt mask value:1:disbale interrupt.0:enable
 *interrupt.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_timer_intmask(uint32_t timer_ch,
                         uint32_t int_type,
                         uint32_t int_mask)
{
  uint32_t tmp_addr;
  uint32_t tmp_val;

  tmp_addr = TIMER_BASE + TIMER_TIER2_OFFSET + 4 * timer_ch;
  tmp_val  = bl602_up_tim_regin(tmp_addr);

  switch (int_type)
    {
    case TIMER_INT_COMP_0:
      if (int_mask == 0)
        {
          /* Enable this interrupt */

          bl602_up_tim_regout(tmp_addr, tmp_val |= 1 << TIMER_TIER_0_POS);
        }
      else
        {
          /* Disable this interrupt */

          bl602_up_tim_regout(tmp_addr, tmp_val &= ~(1 << TIMER_TIER_0_POS));
        }

      break;

    case TIMER_INT_COMP_1:
      if (int_mask == 0)
        {
          /* Enable this interrupt */

          bl602_up_tim_regout(tmp_addr, tmp_val |= 1 << TIMER_TIER_1_POS);
        }
      else
        {
          /* Disable this interrupt */

          bl602_up_tim_regout(tmp_addr, tmp_val &= ~(1 << TIMER_TIER_1_POS));
        }

      break;

    case TIMER_INT_COMP_2:
      if (int_mask == 0)
        {
          /* Enable this interrupt */

          bl602_up_tim_regout(tmp_addr, tmp_val |= 1 << TIMER_TIER_2_POS);
        }
      else
        {
          /* Disable this interrupt */

          bl602_up_tim_regout(tmp_addr, tmp_val &= ~(1 << TIMER_TIER_2_POS));
        }

      break;

    case TIMER_INT_ALL:
      if (int_mask == 0)
        {
          /* Enable this interrupt */

          bl602_up_tim_regout(tmp_addr, tmp_val |= 1 << TIMER_TIER_0_POS);
          bl602_up_tim_regout(tmp_addr, tmp_val |= 1 << TIMER_TIER_1_POS);
          bl602_up_tim_regout(tmp_addr, tmp_val |= 1 << TIMER_TIER_2_POS);
        }
      else
        {
          /* Disable this interrupt */

          bl602_up_tim_regout(tmp_addr, tmp_val &= ~(1 << TIMER_TIER_0_POS));
          bl602_up_tim_regout(tmp_addr, tmp_val &= ~(1 << TIMER_TIER_1_POS));
          bl602_up_tim_regout(tmp_addr, tmp_val &= ~(1 << TIMER_TIER_2_POS));
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

void bl602_wdt_set_clock(uint32_t clk_src, uint8_t div)
{
  uint32_t tmp_val;

  /* Configure watchdog timer clock source */

  tmp_val = bl602_up_tim_regin(TIMER_BASE + TIMER_TCCR_OFFSET);
  tmp_val = bl602_data_setbits(tmp_val, TIMER_CS_WDT_POS, 2, clk_src);
  bl602_up_tim_regout(TIMER_BASE + TIMER_TCCR_OFFSET, tmp_val);

  /* Configure watchdog timer clock divison */

  tmp_val = bl602_up_tim_regin(TIMER_BASE + TIMER_TCDR_OFFSET);
  tmp_val = bl602_data_setbits(tmp_val, TIMER_WCDR_POS, 8, div);
  bl602_up_tim_regout(TIMER_BASE + TIMER_TCDR_OFFSET, tmp_val);
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
 *   Watchdog match comapre register value.
 *
 ****************************************************************************/

uint32_t bl602_wdt_getmatchvalue(void)
{
  uint32_t tmp_val;

  bl602_wdt_access();

  /* Get watchdog timer match register value */

  tmp_val = bl602_up_tim_regin(TIMER_BASE + TIMER_WMR_OFFSET);

  return tmp_val;
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

  bl602_up_tim_regout(TIMER_BASE + TIMER_WMR_OFFSET, val);
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
  uint32_t tmp_val;

  bl602_wdt_access();

  /* Get watchdog timer count register value */

  tmp_val = bl602_up_tim_regin(TIMER_BASE + TIMER_WVR_OFFSET);

  return tmp_val;
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
  uint32_t tmp_val;

  /* Reset watchdog timer count register value */

  bl602_wdt_access();

  tmp_val = bl602_up_tim_regin(TIMER_BASE + TIMER_WCR_OFFSET);

  /* Set watchdog counter reset register bit0 to 1 */

  bl602_up_tim_regout(TIMER_BASE + TIMER_WCR_OFFSET,
                      tmp_val |= 1 << TIMER_WCR_POS);
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

uint32_t bl602_wdt_getresetstatus(void)
{
  uint32_t tmp_val;
  uint32_t ret;

  bl602_wdt_access();

  /* Get watchdog status register */

  tmp_val = bl602_up_tim_regin(TIMER_BASE + TIMER_WSR_OFFSET);

  ret = (((tmp_val) & (1 << (TIMER_WTS_POS))) ? 1 : 0);
  return ret;
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
  uint32_t tmp_val;

  bl602_wdt_access();

  tmp_val = bl602_up_tim_regin(TIMER_BASE + TIMER_WSR_OFFSET);

  /* Set watchdog status register */

  bl602_up_tim_regout(TIMER_BASE + TIMER_WSR_OFFSET,
                      tmp_val &= ~(1 << TIMER_WTS_POS));
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
  uint32_t tmp_val;

  bl602_wdt_access();

  tmp_val = bl602_up_tim_regin(TIMER_BASE + TIMER_WMER_OFFSET);

  bl602_up_tim_regout(TIMER_BASE + TIMER_WMER_OFFSET,
                      tmp_val |= 1 << TIMER_WE_POS);
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
  uint32_t tmp_val;

  bl602_wdt_access();

  tmp_val = bl602_up_tim_regin(TIMER_BASE + TIMER_WMER_OFFSET);

  bl602_up_tim_regout(TIMER_BASE + TIMER_WMER_OFFSET,
                      tmp_val &= ~(1 << TIMER_WE_POS));
}

/****************************************************************************
 * Name: bl602_wdt_intmask
 *
 * Description:
 *   Watchdog timer mask or unmask certain or all interrupt.
 *
 * Input Parameters:
 *   int_type - Watchdog interrupt type.
 *   int_mask - Watchdog interrupt mask value:BL_STD_MASK:disbale
 *interrupt.BL_STD_UNMASK:enable interrupt.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_wdt_intmask(uint32_t int_type, uint32_t int_mask)
{
  uint32_t tmp_val;

  bl602_wdt_access();

  /* Deal with watchdog match/interrupt enable register,WRIE:watchdog
   * reset/interrupt enable
   */

  tmp_val = bl602_up_tim_regin(TIMER_BASE + TIMER_WMER_OFFSET);

  switch (int_type)
    {
    case WDT_INT:
      if (int_mask == 0)
        {
          /* Enable this interrupt */

          /* 0 means generates a watchdog interrupt,a watchdog timer reset is
           * not generated
           */

          bl602_up_tim_regout(TIMER_BASE + TIMER_WMER_OFFSET,
                              tmp_val &= ~(1 << TIMER_WRIE_POS));
        }
      else
        {
          /* Disable this interrupt */

          /* 1 means generates a watchdog timer reset,a watchdog interrupt is
           * not generated
           */

          bl602_up_tim_regout(TIMER_BASE + TIMER_WMER_OFFSET,
                              tmp_val |= 1 << TIMER_WRIE_POS);
        }

      break;
    default:
      break;
    }
}

/****************************************************************************
 * boards/risc-v/bl602/evb/src/bl602_tim.c
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TIMER_MAX_MATCH 3

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_getcompvalue
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

uint32_t timer_getcompvalue(timer_chan_t timer_ch, timer_comp_id_t cmp_no)
{
  uint32_t tmp_val;

  tmp_val = BL_RD_WORD(TIMER_BASE + TIMER_TMR2_0_OFFSET +
                       4 * (TIMER_MAX_MATCH * timer_ch + cmp_no));
  return tmp_val;
}

/****************************************************************************
 * Name: timer_setcompvalue
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

void timer_setcompvalue(timer_chan_t    timer_ch,
                        timer_comp_id_t cmp_no,
                        uint32_t        val)
{
  BL_WR_WORD(TIMER_BASE + TIMER_TMR2_0_OFFSET +
               4 * (TIMER_MAX_MATCH * timer_ch + cmp_no),
             val);
}

/****************************************************************************
 * Name: timer_getcountervalue
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

uint32_t timer_getcountervalue(timer_chan_t timer_ch)
{
  uint32_t tmp_val;
  uint32_t tmp_addr;

  /* TO avoid risk of reading, don't read TCVWR directly
   * request for read
   */

  tmp_addr = TIMER_BASE + TIMER_TCVWR2_OFFSET + 4 * timer_ch;
  BL_WR_WORD(tmp_addr, 1);

  /* Need wait */

  tmp_val = BL_RD_WORD(tmp_addr);
  tmp_val = BL_RD_WORD(tmp_addr);
  tmp_val = BL_RD_WORD(tmp_addr);

  return tmp_val;
}

/****************************************************************************
 * Name: timer_getmatchstatus
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

uint32_t timer_getmatchstatus(timer_chan_t timer_ch, timer_comp_id_t cmp_no)
{
  uint32_t tmp_val;
  uint32_t bit_status = 0;

  tmp_val = BL_RD_WORD(TIMER_BASE + TIMER_TMSR2_OFFSET + 4 * timer_ch);
  switch (cmp_no)
    {
    case TIMER_COMP_ID_0:
      bit_status = BL_IS_REG_BIT_SET(tmp_val, TIMER_TMSR_0) ? 1 : 0;
      break;
    case TIMER_COMP_ID_1:
      bit_status = BL_IS_REG_BIT_SET(tmp_val, TIMER_TMSR_1) ? 1 : 0;
      break;
    case TIMER_COMP_ID_2:
      bit_status = BL_IS_REG_BIT_SET(tmp_val, TIMER_TMSR_2) ? 1 : 0;
      break;
    default:
      break;
    }

  return bit_status;
}

/****************************************************************************
 * Name: timer_getpreloadvalue
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

uint32_t timer_getpreloadvalue(timer_chan_t timer_ch)
{
  uint32_t tmp_val;
  tmp_val = BL_RD_WORD(TIMER_BASE + TIMER_TPLVR2_OFFSET + 4 * timer_ch);

  return tmp_val;
}

/****************************************************************************
 * Name: timer_setpreloadvalue
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

void timer_setpreloadvalue(timer_chan_t timer_ch, uint32_t val)
{
  BL_WR_WORD(TIMER_BASE + TIMER_TPLVR2_OFFSET + 4 * timer_ch, val);
}

/****************************************************************************
 * Name: timer_setpreloadtrigsrc
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

void timer_setpreloadtrigsrc(timer_chan_t         timer_ch,
                             timer_preload_trig_t pl_src)
{
  BL_WR_WORD(TIMER_BASE + TIMER_TPLCR2_OFFSET + 4 * timer_ch, pl_src);
}

/****************************************************************************
 * Name: timer_setcountmode
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

void timer_setcountmode(timer_chan_t timer_ch, timer_countmode_t count_mode)
{
  uint32_t tmpval;

  tmpval = BL_RD_WORD(TIMER_BASE + TIMER_TCMR_OFFSET);
  tmpval &= (~(1 << (timer_ch + 1)));
  tmpval |= (count_mode << (timer_ch + 1));

  BL_WR_WORD(TIMER_BASE + TIMER_TCMR_OFFSET, tmpval);
}

/****************************************************************************
 * Name: timer_clearintstatus
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

void timer_clearintstatus(timer_chan_t timer_ch, timer_comp_id_t cmp_no)
{
  uint32_t tmp_addr;
  uint32_t tmp_val;

  tmp_addr = TIMER_BASE + TIMER_TICR2_OFFSET + 4 * timer_ch;

  tmp_val = BL_RD_WORD(tmp_addr);
  tmp_val |= (1 << cmp_no);

  BL_WR_WORD(tmp_addr, tmp_val);
}

/****************************************************************************
 * Name: timer_init
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

void timer_init(timer_cfg_t *timer_cfg)
{
  timer_chan_t timer_ch = timer_cfg->timer_ch;
  uint32_t     tmp_val;

  /* Configure timer clock source */

  tmp_val = BL_RD_REG(TIMER_BASE, TIMER_TCCR);
  if (timer_ch == TIMER_CH0)
    {
      tmp_val = BL_SET_REG_BITS_VAL(tmp_val, TIMER_CS_1, timer_cfg->clk_src);
    }
  else
    {
      tmp_val = BL_SET_REG_BITS_VAL(tmp_val, TIMER_CS_2, timer_cfg->clk_src);
    }

  BL_WR_REG(TIMER_BASE, TIMER_TCCR, tmp_val);

  /* Configure timer clock division */

  tmp_val = BL_RD_REG(TIMER_BASE, TIMER_TCDR);
  if (timer_ch == TIMER_CH0)
    {
      tmp_val =
        BL_SET_REG_BITS_VAL(tmp_val, TIMER_TCDR2, timer_cfg->clock_division);
    }
  else
    {
      tmp_val =
        BL_SET_REG_BITS_VAL(tmp_val, TIMER_TCDR3, timer_cfg->clock_division);
    }

  BL_WR_REG(TIMER_BASE, TIMER_TCDR, tmp_val);

  /* Configure timer count mode: preload or free run */

  timer_setcountmode(timer_ch, timer_cfg->count_mode);

  /* Configure timer preload trigger src */

  timer_setpreloadtrigsrc(timer_ch, timer_cfg->pl_trig_src);

  if (timer_cfg->count_mode == TIMER_COUNT_PRELOAD)
    {
      /* Configure timer preload value */

      timer_setpreloadvalue(timer_ch, timer_cfg->pre_load_val);
    }

  /* Configure match compare values */

  timer_setcompvalue(timer_ch, TIMER_COMP_ID_0, timer_cfg->match_val0);
  timer_setcompvalue(timer_ch, TIMER_COMP_ID_1, timer_cfg->match_val1);
  timer_setcompvalue(timer_ch, TIMER_COMP_ID_2, timer_cfg->match_val2);
}

/****************************************************************************
 * Name: timer_enable
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

void timer_enable(timer_chan_t timer_ch)
{
  uint32_t tmp_val;

  tmp_val = BL_RD_REG(TIMER_BASE, TIMER_TCER);
  tmp_val |= (1 << (timer_ch + 1));

  BL_WR_REG(TIMER_BASE, TIMER_TCER, tmp_val);
}

/****************************************************************************
 * Name: timer_disable
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

void timer_disable(timer_chan_t timer_ch)
{
  uint32_t tmp_val;

  tmp_val = BL_RD_REG(TIMER_BASE, TIMER_TCER);
  tmp_val &= (~(1 << (timer_ch + 1)));

  BL_WR_REG(TIMER_BASE, TIMER_TCER, tmp_val);
}

/****************************************************************************
 * Name: timer_intmask
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

void timer_intmask(timer_chan_t timer_ch,
                   timer_int_t  int_type,
                   uint32_t     int_mask)
{
  uint32_t tmp_addr;
  uint32_t tmp_val;

  tmp_addr = TIMER_BASE + TIMER_TIER2_OFFSET + 4 * timer_ch;
  tmp_val  = BL_RD_WORD(tmp_addr);

  switch (int_type)
    {
    case TIMER_INT_COMP_0:
      if (int_mask == 0)
        {
          /* Enable this interrupt */

          BL_WR_WORD(tmp_addr, BL_SET_REG_BIT(tmp_val, TIMER_TIER_0));
        }
      else
        {
          /* Disable this interrupt */

          BL_WR_WORD(tmp_addr, BL_CLR_REG_BIT(tmp_val, TIMER_TIER_0));
        }

      break;

    case TIMER_INT_COMP_1:
      if (int_mask == 0)
        {
          /* Enable this interrupt */

          BL_WR_WORD(tmp_addr, BL_SET_REG_BIT(tmp_val, TIMER_TIER_1));
        }
      else
        {
          /* Disable this interrupt */

          BL_WR_WORD(tmp_addr, BL_CLR_REG_BIT(tmp_val, TIMER_TIER_1));
        }

      break;

    case TIMER_INT_COMP_2:
      if (int_mask == 0)
        {
          /* Enable this interrupt */

          BL_WR_WORD(tmp_addr, BL_SET_REG_BIT(tmp_val, TIMER_TIER_2));
        }
      else
        {
          /* Disable this interrupt */

          BL_WR_WORD(tmp_addr, BL_CLR_REG_BIT(tmp_val, TIMER_TIER_2));
        }

      break;

    case TIMER_INT_ALL:
      if (int_mask == 0)
        {
          /* Enable this interrupt */

          BL_WR_WORD(tmp_addr, BL_SET_REG_BIT(tmp_val, TIMER_TIER_0));
          BL_WR_WORD(tmp_addr, BL_SET_REG_BIT(tmp_val, TIMER_TIER_1));
          BL_WR_WORD(tmp_addr, BL_SET_REG_BIT(tmp_val, TIMER_TIER_2));
        }
      else
        {
          /* Disable this interrupt */

          BL_WR_WORD(tmp_addr, BL_CLR_REG_BIT(tmp_val, TIMER_TIER_0));
          BL_WR_WORD(tmp_addr, BL_CLR_REG_BIT(tmp_val, TIMER_TIER_1));
          BL_WR_WORD(tmp_addr, BL_CLR_REG_BIT(tmp_val, TIMER_TIER_2));
        }

      break;

    default:
      break;
    }
}

/****************************************************************************
 * Name: wdt_set_clock
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

void wdt_set_clock(timer_clksrc_t clk_src, uint8_t div)
{
  uint32_t tmp_val;

  /* Configure watchdog timer clock source */

  tmp_val = BL_RD_REG(TIMER_BASE, TIMER_TCCR);
  tmp_val = BL_SET_REG_BITS_VAL(tmp_val, TIMER_CS_WDT, clk_src);
  BL_WR_REG(TIMER_BASE, TIMER_TCCR, tmp_val);

  /* Configure watchdog timer clock divison */

  tmp_val = BL_RD_REG(TIMER_BASE, TIMER_TCDR);
  tmp_val = BL_SET_REG_BITS_VAL(tmp_val, TIMER_WCDR, div);
  BL_WR_REG(TIMER_BASE, TIMER_TCDR, tmp_val);
}

/****************************************************************************
 * Name: wdt_getmatchvalue
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

uint32_t wdt_getmatchvalue(void)
{
  uint32_t tmp_val;

  WDT_ENABLE_ACCESS();

  /* Get watchdog timer match register value */

  tmp_val = BL_RD_REG(TIMER_BASE, TIMER_WMR);

  return tmp_val;
}

/****************************************************************************
 * Name: wdt_setcompvalue
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

void wdt_setcompvalue(uint16_t val)
{
  WDT_ENABLE_ACCESS();

  /* Set watchdog timer match register value */

  BL_WR_REG(TIMER_BASE, TIMER_WMR, val);
}

/****************************************************************************
 * Name: wdt_getcountervalue
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

uint16_t wdt_getcountervalue(void)
{
  uint32_t tmp_val;

  WDT_ENABLE_ACCESS();

  /* Get watchdog timer count register value */

  tmp_val = BL_RD_REG(TIMER_BASE, TIMER_WVR);

  return tmp_val;
}

/****************************************************************************
 * Name: wdt_resetcountervalue
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

void wdt_resetcountervalue(void)
{
  uint32_t tmp_val;

  /* Reset watchdog timer count register value */

  WDT_ENABLE_ACCESS();

  tmp_val = BL_RD_REG(TIMER_BASE, TIMER_WCR);

  /* Set watchdog counter reset register bit0 to 1 */

  BL_WR_REG(TIMER_BASE, TIMER_WCR, BL_SET_REG_BIT(tmp_val, TIMER_WCR));
}

/****************************************************************************
 * Name: wdt_getresetstatus
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

uint32_t wdt_getresetstatus(void)
{
  uint32_t tmp_val;
  uint32_t ret;

  WDT_ENABLE_ACCESS();

  /* Get watchdog status register */

  tmp_val = BL_RD_REG(TIMER_BASE, TIMER_WSR);

  ret = (BL_IS_REG_BIT_SET(tmp_val, TIMER_WTS)) ? 1 : 0;
  return ret;
}

/****************************************************************************
 * Name: wdt_clearresetstatus
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

void wdt_clearresetstatus(void)
{
  uint32_t tmp_val;

  WDT_ENABLE_ACCESS();

  tmp_val = BL_RD_REG(TIMER_BASE, TIMER_WSR);

  /* Set watchdog status register */

  BL_WR_REG(TIMER_BASE, TIMER_WSR, BL_CLR_REG_BIT(tmp_val, TIMER_WTS));
}

/****************************************************************************
 * Name: wdt_enable
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

void wdt_enable(void)
{
  uint32_t tmp_val;

  WDT_ENABLE_ACCESS();

  tmp_val = BL_RD_REG(TIMER_BASE, TIMER_WMER);

  BL_WR_REG(TIMER_BASE, TIMER_WMER, BL_SET_REG_BIT(tmp_val, TIMER_WE));
}

/****************************************************************************
 * Name: wdt_disable
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

void wdt_disable(void)
{
  uint32_t tmp_val;

  WDT_ENABLE_ACCESS();

  tmp_val = BL_RD_REG(TIMER_BASE, TIMER_WMER);

  BL_WR_REG(TIMER_BASE, TIMER_WMER, BL_CLR_REG_BIT(tmp_val, TIMER_WE));
}

/****************************************************************************
 * Name: wdt_intmask
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

void wdt_intmask(wdt_int_t int_type, uint32_t int_mask)
{
  uint32_t tmp_val;

  WDT_ENABLE_ACCESS();

  /* Deal with watchdog match/interrupt enable register,WRIE:watchdog
   * reset/interrupt enable
   */

  tmp_val = BL_RD_REG(TIMER_BASE, TIMER_WMER);

  switch (int_type)
    {
    case WDT_INT:
      if (int_mask == 0)
        {
          /* Enable this interrupt */

          /* 0 means generates a watchdog interrupt,a watchdog timer reset is
           * not generated
           */

          BL_WR_REG(
            TIMER_BASE, TIMER_WMER, BL_CLR_REG_BIT(tmp_val, TIMER_WRIE));
        }
      else
        {
          /* Disable this interrupt */

          /* 1 means generates a watchdog timer reset,a watchdog interrupt is
           * not generated
           */

          BL_WR_REG(
            TIMER_BASE, TIMER_WMER, BL_SET_REG_BIT(tmp_val, TIMER_WRIE));
        }

      break;
    default:
      break;
    }
}


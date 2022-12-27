/****************************************************************************
 * arch/risc-v/src/bl602/bl602_tim.h
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

#ifndef __ARCH_RISCV_SRC_BL602_BL602_TIM_H
#define __ARCH_RISCV_SRC_BL602_BL602_TIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TIMER channel type definition */

#define TIMER_CH0    0 /* TIMER channel 0 port */
#define TIMER_CH1    1 /* TIMER channel 1 port */
#define TIMER_CH_MAX 2

/* TIMER clock source type definition */

#define TIMER_CLKSRC_FCLK 0 /* TIMER clock source :System CLK */
#define TIMER_CLKSRC_32K  1 /* TIMER clock source :32K CLK */
#define TIMER_CLKSRC_1K   2 /* TIMER clock source :1K CLK (not for WDT) */
#define TIMER_CLKSRC_XTAL 3 /* TIMER clock source :XTAL CLK */

/* TIMER match compare ID type definition */

#define TIMER_COMP_ID_0 0 /* TIMER match compare ID 0 */
#define TIMER_COMP_ID_1 1 /* TIMER match compare ID 1 */
#define TIMER_COMP_ID_2 2 /* TIMER match compare ID 2 */
#define TIMER_MAX_MATCH 3

/* TIMER preload source type definition */

#define TIMER_PRELOAD_TRIG_NONE   0 /* TIMER no preload source, free run */
#define TIMER_PRELOAD_TRIG_COMP0  1 /* TIMER count register preload triggered by comparator 0 */
#define TIMER_PRELOAD_TRIG_COMP1  2 /* TIMER count register preload triggered by comparator 1 */
#define TIMER_PRELOAD_TRIG_COMP2  3 /* TIMER count register preload triggered by comparator 2 */

/* TIMER count register run mode type definition */

#define TIMER_COUNT_PRELOAD 0 /* TIMER count register preload from comparator register */
#define TIMER_COUNT_FREERUN 1 /* TIMER count register free run */

/* TIMER interrupt type definition */

#define TIMER_INT_COMP_0 0 /* Comparator 0 match cause interrupt */
#define TIMER_INT_COMP_1 1 /* Comparator 1 match cause interrupt */
#define TIMER_INT_COMP_2 2 /* Comparator 2 match cause interrupt */
#define TIMER_INT_ALL    3

/* Watchdog timer interrupt type definition */

#define WDT_INT     0 /* Comparator 0 match cause interrupt */
#define WDT_INT_ALL 1

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct timer_cfg_s
{
  uint8_t timer_ch; /* Timer channel */
  uint8_t clk_src;  /* Timer clock source */

  /* Timer count register preload trigger source select */

  uint8_t pl_trig_src;

  uint8_t count_mode;      /* Timer count mode */
  uint8_t  clock_division; /* Timer clock division value */
  uint32_t match_val0;     /* Timer match 0 value 0 */
  uint32_t match_val1;     /* Timer match 1 value 0 */
  uint32_t match_val2;     /* Timer match 2 value 0 */
  uint32_t pre_load_val;   /* Timer preload value */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
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

uint32_t bl602_timer_getcompvalue(uint8_t timer_ch, uint8_t cmp_no);

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

void bl602_timer_setcompvalue(uint8_t timer_ch, uint8_t cmp_no,
                              uint32_t val);

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

uint32_t bl602_timer_getcountervalue(uint32_t timer_ch);

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

bool bl602_timer_getmatchstatus(uint32_t timer_ch, uint8_t cmp_no);

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

uint32_t bl602_timer_getpreloadvalue(uint32_t timer_ch);

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

void bl602_timer_setpreloadvalue(uint8_t timer_ch, uint32_t val);

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

void bl602_timer_setpreloadtrigsrc(uint8_t timer_ch, uint32_t pl_src);

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

void bl602_timer_setcountmode(uint32_t timer_ch, uint8_t count_mode);

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

void bl602_timer_clearintstatus(uint8_t timer_ch, uint32_t cmp_no);

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

void bl602_timer_init(struct timer_cfg_s *timer_cfg);

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

void bl602_timer_enable(uint8_t timer_ch);

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

void bl602_timer_disable(uint8_t timer_ch);

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
                         uint8_t int_mask);

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

void bl602_wdt_set_clock(uint8_t clk_src, uint8_t div);

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

uint16_t bl602_wdt_getmatchvalue(void);

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

void bl602_wdt_setcompvalue(uint16_t val);

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

uint16_t bl602_wdt_getcountervalue(void);

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

void bl602_wdt_resetcountervalue(void);

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

bool bl602_wdt_getresetstatus(void);

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

void bl602_wdt_clearresetstatus(void);

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

void bl602_wdt_enable(void);

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

void bl602_wdt_disable(void);

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

void bl602_wdt_intmask(uint8_t int_type, uint8_t int_mask);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_BL602_BL602_TIM_H */

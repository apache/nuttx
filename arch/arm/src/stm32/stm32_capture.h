/****************************************************************************
 * arch/arm/src/stm32/stm32_capture.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_CAPTURE_H
#define __ARCH_ARM_SRC_STM32_STM32_CAPTURE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include <arch/board/board.h>
#include "hardware/stm32_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers ******************************************************************/

#define STM32_CAP_SETCLOCK(d,clk_src,psc,max)   ((d)->ops->setclock(d,clk_src,psc,max))
#define STM32_CAP_SETCHANNEL(d,ch,cfg)          ((d)->ops->setchannel(d,ch,cfg))
#define STM32_CAP_GETCAPTURE(d,ch)              ((d)->ops->getcapture(d,ch))
#define STM32_CAP_SETISR(d,hnd,arg)             ((d)->ops->setisr(d,hnd,arg))
#define STM32_CAP_ENABLEINT(d,s,on)             ((d)->ops->enableint(d,s,on))
#define STM32_CAP_ACKFLAGS(d,f)                 ((d)->ops->ackflags(d,f))
#define STM32_CAP_GETFLAGS(d)                   ((d)->ops->getflags(d))

/****************************************************************************
 * Public Types
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

/* Capture Device Structure */

struct stm32_cap_dev_s
{
  struct stm32_cap_ops_s *ops;
};

/* Capture input EDGE sources */

typedef enum
{
  /* Mapped */

  STM32_CAP_MAPPED_MASK         = (GTIM_CCMR1_CC1S_MASK),
  STM32_CAP_MAPPED_TI1          = (GTIM_CCMR_CCS_CCIN1),
  STM32_CAP_MAPPED_TI2          = (GTIM_CCMR_CCS_CCIN2),

/* TODO STM32_CAP_MAPPED_TRC     = (GTIM_CCMR_CCS_CCINTRC), */

  /* Event prescaler */

  STM32_CAP_INPSC_MASK          = (GTIM_CCMR1_IC1PSC_MASK),
  STM32_CAP_INPSC_NO            = (0 << GTIM_CCMR1_IC1PSC_SHIFT),
  STM32_CAP_INPSC_2EVENTS       = (1 << GTIM_CCMR1_IC1PSC_SHIFT),
  STM32_CAP_INPSC_4EVENTS       = (2 << GTIM_CCMR1_IC1PSC_SHIFT),
  STM32_CAP_INPSC_8EVENTS       = (3 << GTIM_CCMR1_IC1PSC_SHIFT),

  /* Event prescaler */

  STM32_CAP_FILTER_MASK         = (GTIM_CCMR1_IC1F_MASK),
  STM32_CAP_FILTER_NO           = (0 << GTIM_CCMR1_IC1F_SHIFT),

  /* Internal clock with N time to confirm event */

  STM32_CAP_FILTER_INT_N2       = (1 << GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_INT_N4       = (2 << GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_INT_N8       = (3 << GTIM_CCMR1_IC1F_SHIFT),

  /* DTS clock div by D with N time to confirm event */

  STM32_CAP_FILTER_DTS_D2_N6    = (4 << GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D2_N8    = (5 << GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D4_N6    = (6 << GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D4_N8    = (7 << GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D8_N6    = (8 << GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D8_N8    = (9 << GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D16_N5   = (10 << GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D16_N6   = (11 << GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D16_N8   = (12 << GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D32_N5   = (13 << GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D32_N6   = (14 << GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D32_N8   = (15 << GTIM_CCMR1_IC1F_SHIFT),

  /* EDGE */

  STM32_CAP_EDGE_MASK           = (3 << 8),
  STM32_CAP_EDGE_DISABLED       = (0 << 8),
  STM32_CAP_EDGE_RISING         = (1 << 8),
  STM32_CAP_EDGE_FALLING        = (2 << 8),
  STM32_CAP_EDGE_BOTH           = (3 << 8),
} stm32_cap_ch_cfg_t;

/* Capture clock sources */

typedef enum
{
  STM32_CAP_CLK_INT = 0,
  STM32_CAP_CLK_EXT,

  /* TODO: Add other clock */
} stm32_cap_clk_t;

/* Capture flags */

typedef enum
{
  /* One of the following */

  STM32_CAP_FLAG_IRQ_COUNTER    = (GTIM_SR_UIF),

  STM32_CAP_FLAG_IRQ_CH_1       = (GTIM_SR_CC1IF),
  STM32_CAP_FLAG_IRQ_CH_2       = (GTIM_SR_CC2IF),
  STM32_CAP_FLAG_IRQ_CH_3       = (GTIM_SR_CC3IF),
  STM32_CAP_FLAG_IRQ_CH_4       = (GTIM_SR_CC4IF),

  STM32_CAP_FLAG_OF_CH_1        = (GTIM_SR_CC1OF),
  STM32_CAP_FLAG_OF_CH_2        = (GTIM_SR_CC2OF),
  STM32_CAP_FLAG_OF_CH_3        = (GTIM_SR_CC3OF),
  STM32_CAP_FLAG_OF_CH_4        = (GTIM_SR_CC4OF)
} stm32_cap_flags_t;

#define STM32_CAP_FLAG_IRQ_CH(ch)   (GTIM_SR_CC1IF<<((ch)-1))
#define STM32_CAP_FLAG_OF_CH(ch)    (GTIM_SR_CC1OF<<((ch)-1))
#define STM32_CAP_CHANNEL_COUNTER   0

/* Capture Operations */

struct stm32_cap_ops_s
{
  int  (*setclock)(struct stm32_cap_dev_s *dev, stm32_cap_clk_t clk,
                   uint32_t prescaler, uint32_t max);
  int  (*setchannel)(struct stm32_cap_dev_s *dev, uint8_t channel,
                     stm32_cap_ch_cfg_t cfg);
  uint32_t (*getcapture)(struct stm32_cap_dev_s *dev, uint8_t channel);
  int  (*setisr)(struct stm32_cap_dev_s *dev, xcpt_t handler, void *arg);
  void (*enableint)(struct stm32_cap_dev_s *dev, stm32_cap_flags_t src,
                    bool on);
  void (*ackflags)(struct stm32_cap_dev_s *dev, int flags);
  stm32_cap_flags_t (*getflags)(struct stm32_cap_dev_s *dev);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Power-up timer and get its structure */

struct stm32_cap_dev_s *stm32_cap_init(int timer);

/* Power-down timer, mark it as unused */

int stm32_cap_deinit(struct stm32_cap_dev_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32_STM32_CAPTURE_H */

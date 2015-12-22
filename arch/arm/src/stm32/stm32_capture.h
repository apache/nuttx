/************************************************************************************
 * arch/arm/src/stm32/stm32_capture.h
 *
 *   Copyright (C) 2015 Bouteville Pierre-Noel. All rights reserved.
 *   Author: Bouteville Pierre-Noel <pnb990@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_STM32_CAPTURE_H
#define __ARCH_ARM_SRC_STM32_STM32_CAPTURE_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>


#include "chip.h"
#include <arch/board/board.h>
#include "chip/stm32_tim.h"


/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Helpers **************************************************************************/

#define STM32_TIM_SETCLOCK(d,freq)          ((d)->ops->setclock(d,freq))
#define STM32_TIM_SETCHANNEL(d,ch,mode)     ((d)->ops->setchannel(d,ch,mode))
#define STM32_TIM_GETCAPTURE(d,ch)          ((d)->ops->getcapture(d,ch))
#define STM32_TIM_SETISR(d,hnd,s)           ((d)->ops->setisr(d,hnd,s))
#define STM32_TIM_ENABLEINT(d,s,on)         ((d)->ops->enableint(d,s,on))
#define STM32_TIM_ACKFLAGS(d,s)             ((d)->ops->ackflags(d,s))
#define STM32_TIM_GETFLAGS(d)               ((d)->ops->getflags(d))

/************************************************************************************
 * Public Types
 ************************************************************************************/

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
  STM32_CAP_MAPPED_TI1          = (1<<GTIM_CCMR1_CC1S_SHIFT),
  STM32_CAP_MAPPED_TI2          = (2<<GTIM_CCMR1_CC1S_SHIFT),
/*TODO STM32_CAP_MAPPED_TRC     = (3<<GTIM_CCMR1_CC1S_SHIFT),*/
                                
    /* Event prescaler */
  STM32_CAP_INPSC_MASK          = (GTIM_CCMR1_IC1PSC_MASK),
  STM32_CAP_INPSC_NO            = (0<<GTIM_CCMR1_IC1PSC_SHIFT),
  STM32_CAP_INPSC_2EVENTS       = (1<<GTIM_CCMR1_IC1PSC_SHIFT),
  STM32_CAP_INPSC_4EVENTS       = (2<<GTIM_CCMR1_IC1PSC_SHIFT),
  STM32_CAP_INPSC_8EVENTS       = (3<<GTIM_CCMR1_IC1PSC_SHIFT),

    /* Event prescaler */
  STM32_CAP_FILTER_MASK         = (GTIM_CCMR1_IC1F_MASK),
  STM32_CAP_FILTER_NO           = (0<<GTIM_CCMR1_IC1F_SHIFT),
  /* internal clock with N time to confirm event */
  STM32_CAP_FILTER_INT_N2       = (1<<GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_INT_N4       = (2<<GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_INT_N8       = (3<<GTIM_CCMR1_IC1F_SHIFT),
  /* DTS clock div by D with N time to confirm event */
  STM32_CAP_FILTER_DTS_D2_N6    = (4<<GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D2_N8    = (5<<GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D4_N6    = (6<<GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D4_N8    = (7<<GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D8_N6    = (8<<GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D8_N8    = (9<<GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D16_N5   = (10<<GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D16_N6   = (11<<GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D16_N8   = (12<<GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D32_N5   = (13<<GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D32_N6   = (14<<GTIM_CCMR1_IC1F_SHIFT),
  STM32_CAP_FILTER_DTS_D32_N8   = (15<<GTIM_CCMR1_IC1F_SHIFT),

    /* EDGE */
  STM32_CAP_EDGE_MASK           = (3<<8),
  STM32_CAP_EDGE_DISABLED       = (0<<8),
  STM32_CAP_EDGE_RISING         = (1<<8),
  STM32_CAP_EDGE_FALLING        = (2<<8),
  STM32_CAP_EDGE_BOTH           = (3<<8),

} stm32_cap_ch_cfg_t;

/* TIM clock sources */

typedef enum
{

  STM32_CAP_CLK_INT= 0,
  STM32_CAP_CLK_EXT,

  /* TODO: Add other clock */
  
} stm32_cap_clk_t;

/* TIM Sources */

typedef enum
{
  /* One of the following */

  STM32_CAP_FLAG_IRQ_TIMER      = (GTIM_SR_UIF),

  STM32_CAP_FLAG_IRQ_CH_1       = (GTIM_SR_CC1IF),
  STM32_CAP_FLAG_IRQ_CH_2       = (GTIM_SR_CC2IF),
  STM32_CAP_FLAG_IRQ_CH_3       = (GTIM_SR_CC3IF),
  STM32_CAP_FLAG_IRQ_CH_4       = (GTIM_SR_CC4IF),

  STM32_CAP_FLAG_OF_CH_1        = (GTIM_SR_CC1OF),
  STM32_CAP_FLAG_OF_CH_2        = (GTIM_SR_CC2OF),
  STM32_CAP_FLAG_OF_CH_3        = (GTIM_SR_CC3OF),
  STM32_CAP_FLAG_OF_CH_4        = (GTIM_SR_CC4OF)

} stm32_cap_flags_t;

        
/* TIM Operations */

struct stm32_cap_ops_s
{
  int  (*setclock)(FAR struct stm32_cap_dev_s *dev, stm32_cap_clk_src_t clk_src, 
                   uint32_t prescaler);
  int  (*setchannel)(FAR struct stm32_cap_dev_s *dev, uint8_t channel);
  int  (*setisr)(FAR struct stm32_tim_dev_s *dev,xcpt_t handler); 
  void (*enableint)(FAR struct stm32_tim_dev_s *dev, stm32_cap_flags_t src, bool on );
};

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/* Power-up timer and get its structure */

FAR struct stm32_cap_dev_s *stm32_cap_init(int timer);

/* Power-down timer, mark it as unused */

int stm32_cap_deinit(FAR struct stm32_cap_dev_s * dev);


#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32_STM32_TIM_H */

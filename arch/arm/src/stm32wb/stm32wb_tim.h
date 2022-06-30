/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_tim.h
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

#ifndef __ARCH_ARM_SRC_STM32WB_STM32WB_TIM_H
#define __ARCH_ARM_SRC_STM32WB_STM32WB_TIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/stm32wb_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Generalized register definitions */

#define GTIM_CR1_CEN            TIM1_CR1_CEN
#define GTIM_CR1_OPM            TIM1_CR1_OPM
#define GTIM_CR1_ARPE           TIM1_CR1_ARPE
#define GTIM_DIER_UIE           TIM1_DIER_UIE
#define GTIM_SR_UIF             TIM1_SR_UIF
#define GTIM_EGR_UG             TIM1_EGR_UG
#define TIM_1_2_CR1_DIR         TIM1_CR1_DIR
#define TIM_1_2_CR1_CMS_CNTR1   TIM1_CR1_CMS_CNTR1

#define GTIM_CCER_CCX_SHIFT(ch) ((ch) << 2)         /* 4-bits shift
                                                     * per channel */
#define GTIM_CCMR_OCX_SHIFT(ch) (((ch) & 0x1) << 3) /* 8-bits shift
                                                     * per channel */

#define GTIM_CCER_CCXE(ch)      (TIM1_CCER_CC1E << GTIM_CCER_CCX_SHIFT(ch))
#define GTIM_CCER_CCXP(ch)      (TIM1_CCER_CC1P << GTIM_CCER_CCX_SHIFT(ch))
#define GTIM_CCMR_OCXPE(ch)     (TIM1_CCMR1_OC1PE << GTIM_CCMR_OCX_SHIFT(ch))
#define GTIM_CCMR_OCXM_MASK(ch) (TIM1_CCMR1_OC1M_MASK << \
                                GTIM_CCMR_OCX_SHIFT(ch))
#define GTIM_CCMR_OCXM_FRZN(ch) (TIM1_CCMR1_OC1M_FRZN << \
                                GTIM_CCMR_OCX_SHIFT(ch))
#define GTIM_CCMR_OCXM_PWM1(ch) (TIM1_CCMR1_OC1M_PWM1 << \
                                GTIM_CCMR_OCX_SHIFT(ch))

/* Helpers ******************************************************************/

#define STM32WB_TIM_SETMODE(d,mode)       ((d)->ops->setmode(d,mode))
#define STM32WB_TIM_SETFREQ(d,freq)       ((d)->ops->setfreq(d,freq))
#define STM32WB_TIM_SETCLOCK(d,freq)      ((d)->ops->setclock(d,freq))
#define STM32WB_TIM_GETCLOCK(d)           ((d)->ops->getclock(d))
#define STM32WB_TIM_SETPERIOD(d,period)   ((d)->ops->setperiod(d,period))
#define STM32WB_TIM_GETPERIOD(d)          ((d)->ops->getperiod(d))
#define STM32WB_TIM_GETCOUNTER(d)         ((d)->ops->getcounter(d))
#define STM32WB_TIM_GETWIDTH(d)           ((d)->ops->getwidth(d))
#define STM32WB_TIM_SETCHANNEL(d,ch,mode) ((d)->ops->setchannel(d,ch,mode))
#define STM32WB_TIM_SETCOMPARE(d,ch,comp) ((d)->ops->setcompare(d,ch,comp))
#define STM32WB_TIM_GETCAPTURE(d,ch)      ((d)->ops->getcapture(d,ch))
#define STM32WB_TIM_SETISR(d,hnd,arg,s)   ((d)->ops->setisr(d,hnd,arg,s))
#define STM32WB_TIM_ENABLEINT(d,s)        ((d)->ops->enableint(d,s))
#define STM32WB_TIM_DISABLEINT(d,s)       ((d)->ops->disableint(d,s))
#define STM32WB_TIM_ACKINT(d,s)           ((d)->ops->ackint(d,s))
#define STM32WB_TIM_CHECKINT(d,s)         ((d)->ops->checkint(d,s))
#define STM32WB_TIM_ENABLE(d)             ((d)->ops->enable(d))
#define STM32WB_TIM_DISABLE(d)            ((d)->ops->disable(d))
#define STM32WB_TIM_DUMPREGS(d)           ((d)->ops->dump_regs(d))

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

/* TIM Device Structure */

struct stm32wb_tim_dev_s
{
  struct stm32wb_tim_ops_s *ops;
};

/* TIM Modes of Operation */

enum stm32wb_tim_mode_e
{
  STM32WB_TIM_MODE_UNUSED       = -1,

  /* One of the following */

  STM32WB_TIM_MODE_MASK         = 0x0310,
  STM32WB_TIM_MODE_DISABLED     = 0x0000,
  STM32WB_TIM_MODE_UP           = 0x0100,
  STM32WB_TIM_MODE_DOWN         = 0x0110,
  STM32WB_TIM_MODE_UPDOWN       = 0x0200,
  STM32WB_TIM_MODE_PULSE        = 0x0300,

  /* One of the following */

  STM32WB_TIM_MODE_CK_INT       = 0x0000,
#if 0
  STM32WB_TIM_MODE_CK_INT_TRIG  = 0x0400,
  STM32WB_TIM_MODE_CK_EXT       = 0x0800,
  STM32WB_TIM_MODE_CK_EXT_TRIG  = 0x0c00,
#endif

  /* Clock sources, OR'ed with CK_EXT */

#if 0
  STM32WB_TIM_MODE_CK_CHINVALID = 0x0000,
  STM32WB_TIM_MODE_CK_CH1       = 0x0001,
  STM32WB_TIM_MODE_CK_CH2       = 0x0002,
  STM32WB_TIM_MODE_CK_CH3       = 0x0003,
  STM32WB_TIM_MODE_CK_CH4       = 0x0004
#endif

  /* Todo: external trigger block */
};

/* TIM Channel Modes */

enum stm32wb_tim_channel_e
{
  STM32WB_TIM_CH_DISABLED       = 0x00,

  /* Common configuration */

  STM32WB_TIM_CH_POLARITY_POS   = 0x00,
  STM32WB_TIM_CH_POLARITY_NEG   = 0x01,

  /* MODES: */

  STM32WB_TIM_CH_MODE_MASK      = 0x06,

  /* Output Compare Modes */

  STM32WB_TIM_CH_OUTPWM         = 0x04,  /* Enable standard PWM mode, active
                                          * high when counter < compare
                                          */
#if 0
  STM32WB_TIM_CH_OUTCOMPARE     = 0x06,
#endif

  /* TODO other modes ... as PWM capture, ENCODER and Hall Sensor */

#if 0
  STM32WB_TIM_CH_INCAPTURE      = 0x10,
  STM32WB_TIM_CH_INPWM          = 0x20
  STM32WB_TIM_CH_DRIVE_OC                /* Open collector mode */
#endif
};

/* TIM Operations */

struct stm32wb_tim_ops_s
{
  void     (*enable)(struct stm32wb_tim_dev_s *dev);
  void     (*disable)(struct stm32wb_tim_dev_s *dev);
  int      (*setmode)(struct stm32wb_tim_dev_s *dev,
                      enum stm32wb_tim_mode_e mode);
  int      (*setfreq)(struct stm32wb_tim_dev_s *dev, uint32_t freq);
  int      (*setclock)(struct stm32wb_tim_dev_s *dev, uint32_t freq);
  uint32_t (*getclock)(struct stm32wb_tim_dev_s *dev);
  void     (*setperiod)(struct stm32wb_tim_dev_s *dev, uint32_t period);
  uint32_t (*getperiod)(struct stm32wb_tim_dev_s *dev);
  uint32_t (*getcounter)(struct stm32wb_tim_dev_s *dev);
  uint32_t (*getwidth)(struct stm32wb_tim_dev_s *dev);
  int      (*setchannel)(struct stm32wb_tim_dev_s *dev, uint8_t channel,
                         enum stm32wb_tim_channel_e mode);
  int      (*setcompare)(struct stm32wb_tim_dev_s *dev, uint8_t channel,
                         uint32_t compare);
  uint32_t (*getcapture)(struct stm32wb_tim_dev_s *dev, uint8_t channel);

  /* Timer interrupts */

  int      (*setisr)(struct stm32wb_tim_dev_s *dev,
                     xcpt_t handler, void *arg, int source);
  void     (*enableint)(struct stm32wb_tim_dev_s *dev, int source);
  void     (*disableint)(struct stm32wb_tim_dev_s *dev, int source);
  void     (*ackint)(struct stm32wb_tim_dev_s *dev, int source);
  int      (*checkint)(struct stm32wb_tim_dev_s *dev, int source);

  /* Debug */

  void     (*dump_regs)(struct stm32wb_tim_dev_s *dev);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Power-up timer and get its structure */

struct stm32wb_tim_dev_s *stm32wb_tim_init(int timer);

/* Power-down timer, mark it as unused */

int stm32wb_tim_deinit(struct stm32wb_tim_dev_s *dev);

/****************************************************************************
 * Name: stm32wb_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the timer device. This should be of the form
 *             /dev/timer0
 *   timer - the timer number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int stm32wb_timer_initialize(const char *devpath, int timer);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32WB_STM32WB_TIM_H */

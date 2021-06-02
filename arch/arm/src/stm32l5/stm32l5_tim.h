/****************************************************************************
 * arch/arm/src/stm32l5/stm32l5_tim.h
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

#ifndef __ARCH_ARM_SRC_STM32L5_STM32L5_TIM_H
#define __ARCH_ARM_SRC_STM32L5_STM32L5_TIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/stm32l5_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers ******************************************************************/

#define STM32L5_TIM_SETMODE(d,mode)       ((d)->ops->setmode(d,mode))
#define STM32L5_TIM_SETCLOCK(d,freq)      ((d)->ops->setclock(d,freq))
#define STM32L5_TIM_GETCLOCK(d)           ((d)->ops->getclock(d))
#define STM32L5_TIM_SETPERIOD(d,period)   ((d)->ops->setperiod(d,period))
#define STM32L5_TIM_GETPERIOD(d)          ((d)->ops->getperiod(d))
#define STM32L5_TIM_GETCOUNTER(d)         ((d)->ops->getcounter(d))
#define STM32L5_TIM_SETCHANNEL(d,ch,mode) ((d)->ops->setchannel(d,ch,mode))
#define STM32L5_TIM_SETCOMPARE(d,ch,comp) ((d)->ops->setcompare(d,ch,comp))
#define STM32L5_TIM_GETCAPTURE(d,ch)      ((d)->ops->getcapture(d,ch))
#define STM32L5_TIM_SETISR(d,hnd,arg,s)   ((d)->ops->setisr(d,hnd,arg,s))
#define STM32L5_TIM_ENABLEINT(d,s)        ((d)->ops->enableint(d,s))
#define STM32L5_TIM_DISABLEINT(d,s)       ((d)->ops->disableint(d,s))
#define STM32L5_TIM_ACKINT(d,s)           ((d)->ops->ackint(d,s))
#define STM32L5_TIM_CHECKINT(d,s)         ((d)->ops->checkint(d,s))

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

struct stm32l5_tim_dev_s
{
  struct stm32l5_tim_ops_s *ops;
};

/* TIM Modes of Operation */

enum stm32l5_tim_mode_e
{
  STM32L5_TIM_MODE_UNUSED       = -1,

  /* One of the following */

  STM32L5_TIM_MODE_MASK         = 0x0310,
  STM32L5_TIM_MODE_DISABLED     = 0x0000,
  STM32L5_TIM_MODE_UP           = 0x0100,
  STM32L5_TIM_MODE_DOWN         = 0x0110,
  STM32L5_TIM_MODE_UPDOWN       = 0x0200,
  STM32L5_TIM_MODE_PULSE        = 0x0300,

  /* One of the following */

  STM32L5_TIM_MODE_CK_INT       = 0x0000,
#if 0
  STM32L5_TIM_MODE_CK_INT_TRIG  = 0x0400,
  STM32L5_TIM_MODE_CK_EXT       = 0x0800,
  STM32L5_TIM_MODE_CK_EXT_TRIG  = 0x0c00,
#endif

  /* Clock sources, OR'ed with CK_EXT */

#if 0
  STM32L5_TIM_MODE_CK_CHINVALID = 0x0000,
  STM32L5_TIM_MODE_CK_CH1       = 0x0001,
  STM32L5_TIM_MODE_CK_CH2       = 0x0002,
  STM32L5_TIM_MODE_CK_CH3       = 0x0003,
  STM32L5_TIM_MODE_CK_CH4       = 0x0004
#endif

  /* Todo: external trigger block */
};

/* TIM Channel Modes */

enum stm32l5_tim_channel_e
{
  STM32L5_TIM_CH_DISABLED       = 0x00,

  /* Common configuration */

  STM32L5_TIM_CH_POLARITY_POS   = 0x00,
  STM32L5_TIM_CH_POLARITY_NEG   = 0x01,

  /* MODES: */

  STM32L5_TIM_CH_MODE_MASK      = 0x06,

  /* Output Compare Modes */

  STM32L5_TIM_CH_OUTPWM         = 0x04,  /* Enable standard PWM mode, active high when counter < compare */
#if 0
  STM32L5_TIM_CH_OUTCOMPARE     = 0x06,
#endif

  /* TODO other modes ... as PWM capture, ENCODER and Hall Sensor */

#if 0
  STM32L5_TIM_CH_INCAPTURE      = 0x10,
  STM32L5_TIM_CH_INPWM          = 0x20
  STM32L5_TIM_CH_DRIVE_OC   -- open collector mode
#endif
};

/* TIM Operations */

struct stm32l5_tim_ops_s
{
  /* Basic Timers */

  int  (*setmode)(FAR struct stm32l5_tim_dev_s *dev,
                  enum stm32l5_tim_mode_e mode);
  int  (*setclock)(FAR struct stm32l5_tim_dev_s *dev, uint32_t freq);
  uint32_t (*getclock)(FAR struct stm32l5_tim_dev_s *dev);
  void (*setperiod)(FAR struct stm32l5_tim_dev_s *dev, uint32_t period);
  uint32_t (*getperiod)(FAR struct stm32l5_tim_dev_s *dev);
  uint32_t (*getcounter)(FAR struct stm32l5_tim_dev_s *dev);

  /* General and Advanced Timers Adds */

  int  (*setchannel)(FAR struct stm32l5_tim_dev_s *dev, uint8_t channel,
                     enum stm32l5_tim_channel_e mode);
  int  (*setcompare)(FAR struct stm32l5_tim_dev_s *dev, uint8_t channel,
                     uint32_t compare);
  int  (*getcapture)(FAR struct stm32l5_tim_dev_s *dev, uint8_t channel);

  /* Timer interrupts */

  int  (*setisr)(FAR struct stm32l5_tim_dev_s *dev,
                 xcpt_t handler, void *arg, int source);
  void (*enableint)(FAR struct stm32l5_tim_dev_s *dev, int source);
  void (*disableint)(FAR struct stm32l5_tim_dev_s *dev, int source);
  void (*ackint)(FAR struct stm32l5_tim_dev_s *dev, int source);
  int  (*checkint)(FAR struct stm32l5_tim_dev_s *dev, int source);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Power-up timer and get its structure */

FAR struct stm32l5_tim_dev_s *stm32l5_tim_init(int timer);

/* Power-down timer, mark it as unused */

int stm32l5_tim_deinit(FAR struct stm32l5_tim_dev_s *dev);

/****************************************************************************
 * Name: stm32l5_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the timer device.  This should be of the form
 *             /dev/timer0
 *   timer - the timer number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int stm32l5_timer_initialize(FAR const char *devpath, int timer);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32L5_STM32L5_TIM_H */

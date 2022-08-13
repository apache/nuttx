/****************************************************************************
 * arch/arm/src/stm32f7/stm32_tim.h
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

#ifndef __ARCH_ARM_SRC_STM32F7_STM32_TIM_H
#define __ARCH_ARM_SRC_STM32F7_STM32_TIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/stm32_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers ******************************************************************/

#define STM32_TIM_SETMODE(d,mode)       ((d)->ops->setmode(d,mode))
#define STM32_TIM_SETCLOCK(d,freq)      ((d)->ops->setclock(d,freq))
#define STM32_TIM_SETPERIOD(d,period)   ((d)->ops->setperiod(d,period))
#define STM32_TIM_GETCOUNTER(d)         ((d)->ops->getcounter(d))
#define STM32_TIM_SETCOUNTER(d,c)       ((d)->ops->setcounter(d,c))
#define STM32_TIM_GETWIDTH(d)           ((d)->ops->getwidth(d))
#define STM32_TIM_SETCHANNEL(d,ch,mode) ((d)->ops->setchannel(d,ch,mode))
#define STM32_TIM_SETCOMPARE(d,ch,comp) ((d)->ops->setcompare(d,ch,comp))
#define STM32_TIM_GETCAPTURE(d,ch)      ((d)->ops->getcapture(d,ch))
#define STM32_TIM_SETISR(d,hnd,arg,s)   ((d)->ops->setisr(d,hnd,arg,s))
#define STM32_TIM_ENABLEINT(d,s)        ((d)->ops->enableint(d,s))
#define STM32_TIM_DISABLEINT(d,s)       ((d)->ops->disableint(d,s))
#define STM32_TIM_ACKINT(d,s)           ((d)->ops->ackint(d,s))
#define STM32_TIM_CHECKINT(d,s)         ((d)->ops->checkint(d,s))

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

struct stm32_tim_dev_s
{
  struct stm32_tim_ops_s *ops;
};

/* TIM Modes of Operation */

typedef enum
{
  STM32_TIM_MODE_UNUSED       = -1,

  /* One of the following */

  STM32_TIM_MODE_MASK         = 0x0310,
  STM32_TIM_MODE_DISABLED     = 0x0000,
  STM32_TIM_MODE_UP           = 0x0100,
  STM32_TIM_MODE_DOWN         = 0x0110,
  STM32_TIM_MODE_UPDOWN       = 0x0200,
  STM32_TIM_MODE_PULSE        = 0x0300,

  /* One of the following */

  STM32_TIM_MODE_CK_INT       = 0x0000,
#if 0
  STM32_TIM_MODE_CK_INT_TRIG  = 0x0400,   /* TODO: Not supported */
  STM32_TIM_MODE_CK_EXT       = 0x0800,   /* TODO: Not supported */
  STM32_TIM_MODE_CK_EXT_TRIG  = 0x0c00,   /* TODO: Not supported */
#endif

  /* Clock sources, OR'ed with CK_EXT */
#if 0
  STM32_TIM_MODE_CK_CHINVALID = 0x0000,   /* TODO: Not supported */
  STM32_TIM_MODE_CK_CH1       = 0x0001,   /* TODO: Not supported */
  STM32_TIM_MODE_CK_CH2       = 0x0002,   /* TODO: Not supported */
  STM32_TIM_MODE_CK_CH3       = 0x0003,   /* TODO: Not supported */
  STM32_TIM_MODE_CK_CH4       = 0x0004    /* TODO: Not supported */
#endif

  /* TODO: external trigger block */
} stm32_tim_mode_t;

/* TIM Channel Modes */

typedef enum
{
  STM32_TIM_CH_DISABLED       = 0x00,

  /* Common configuration */

  STM32_TIM_CH_POLARITY_POS   = 0x00,
  STM32_TIM_CH_POLARITY_NEG   = 0x01,

  /* MODES: */

  STM32_TIM_CH_MODE_MASK      = 0x0e,

  /* Output Compare Modes */

  STM32_TIM_CH_OUTPWM         = 0x04,     /* Enable standard PWM mode, active high when counter < compare */
#if 0
  STM32_TIM_CH_OUTCOMPARE     = 0x06,     /* TODO: Not supported */
#endif
  STM32_TIM_CH_OUTTOGGLE      = 0x08,

#if 0
  /* TODO other modes ... as PWM capture, ENCODER and Hall Sensor */

  STM32_TIM_CH_INCAPTURE      = 0x10,
  STM32_TIM_CH_INPWM          = 0x20
  STM32_TIM_CH_DRIVE_OC /* -- open collector mode */
#endif
} stm32_tim_channel_t;

/* TIM Operations */

struct stm32_tim_ops_s
{
  /* Basic Timers */

  int  (*setmode)(struct stm32_tim_dev_s *dev, stm32_tim_mode_t mode);
  int  (*setclock)(struct stm32_tim_dev_s *dev, uint32_t freq);
  void (*setperiod)(struct stm32_tim_dev_s *dev, uint32_t period);
  uint32_t (*getcounter)(struct stm32_tim_dev_s *dev);
  void (*setcounter)(struct stm32_tim_dev_s *dev, uint32_t count);

  /* General and Advanced Timers Adds */

  int  (*getwidth)(struct stm32_tim_dev_s *dev);
  int  (*setchannel)(struct stm32_tim_dev_s *dev, uint8_t channel,
                     stm32_tim_channel_t mode);
  int  (*setcompare)(struct stm32_tim_dev_s *dev, uint8_t channel,
                     uint32_t compare);
  int  (*getcapture)(struct stm32_tim_dev_s *dev, uint8_t channel);

  /* Timer interrupts */

  int  (*setisr)(struct stm32_tim_dev_s *dev, xcpt_t handler,
                 void * arg, int source);
  void (*enableint)(struct stm32_tim_dev_s *dev, int source);
  void (*disableint)(struct stm32_tim_dev_s *dev, int source);
  void (*ackint)(struct stm32_tim_dev_s *dev, int source);
  int  (*checkint)(struct stm32_tim_dev_s *dev, int source);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Power-up timer and get its structure */

struct stm32_tim_dev_s *stm32_tim_init(int timer);

/* Power-down timer, mark it as unused */

int stm32_tim_deinit(struct stm32_tim_dev_s *dev);

/****************************************************************************
 * Name: stm32_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the timer device. This should be of the
 *             form /dev/timer0
 *   timer   - the timer number.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int stm32_timer_initialize(const char *devpath, int timer);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32F7_STM32_TIM_H */

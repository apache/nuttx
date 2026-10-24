/****************************************************************************
 * arch/arm/src/common/stm32/stm32_lptim.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_STM32_LPTIM_H
#define __ARCH_ARM_SRC_COMMON_STM32_STM32_LPTIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include <stdbool.h>
#include <stdint.h>

#include "chip.h"
#include "hardware/stm32_lptim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_LPTIM_SETMODE(d,mode)     ((d)->ops->setmode(d,mode))
#define STM32_LPTIM_SETCLOCK(d,freq)    ((d)->ops->setclock(d,freq))
#define STM32_LPTIM_SETCHANNEL(d,ch,en) ((d)->ops->setchannel(d,ch,en))
#define STM32_LPTIM_SETCLOCKSOURCE(d,s) ((d)->ops->setclocksource(d,s))
#define STM32_LPTIM_SETPOLARITY(d,p)    ((d)->ops->setpolarity(d,p))
#define STM32_LPTIM_GETCOUNTER(d)       ((d)->ops->getcounter(d))
#define STM32_LPTIM_SETCOUNTMODE(d,m)   ((d)->ops->setcountmode(d,m))
#define STM32_LPTIM_SETPERIOD(d,period) ((d)->ops->setperiod(d,period))
#define STM32_LPTIM_GETPERIOD(d)        ((d)->ops->getperiod(d))
#define STM32_LPTIM_SETCFGR(d,regval)   ((d)->ops->setcfgr(d,regval))
#define STM32_LPTIM_SETCOMPARE(d,cmp)   ((d)->ops->setcompare(d,cmp))
#define STM32_LPTIM_SETINPUT(d,in,mux)  ((d)->ops->setinput(d,in,mux))
#define STM32_LPTIM_ENABLE(d,on)        ((d)->ops->enable(d,on))
#define STM32_LPTIM_START(d,mode)       ((d)->ops->start(d,mode))
#define STM32_LPTIM_RESETCOUNTER(d)     ((d)->ops->resetcounter(d))
#define STM32_LPTIM_ENABLERAR(d,on)     ((d)->ops->enablerar(d,on))
#define STM32_LPTIM_SETISR(d,hnd,arg,s) ((d)->ops->setisr(d,hnd,arg,s))
#define STM32_LPTIM_ENABLEINT(d,s)      ((d)->ops->enableint(d,s))
#define STM32_LPTIM_DISABLEINT(d,s)     ((d)->ops->disableint(d,s))
#define STM32_LPTIM_ACKINT(d,s)         ((d)->ops->ackint(d,s))

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#if defined(__cplusplus)
extern "C"
{
#endif

typedef enum
{
  STM32_LPTIM_MODE_UNUSED     = -1,
  STM32_LPTIM_MODE_DISABLED   = 0,
  STM32_LPTIM_MODE_SINGLE     = 1,
  STM32_LPTIM_MODE_CONTINUOUS = 2,
  STM32_LPTIM_MODE_MASK       = 15
} stm32_lptim_mode_t;

/* Counter clock: STM32_LPTIM_CLK_EXT is the LPTIM input 1, all other
 * values select the kernel clock chosen in RCC.
 */

typedef enum
{
  STM32_LPTIM_CLK_PCLK = 0,
  STM32_LPTIM_CLK_LSI  = 1,
  STM32_LPTIM_CLK_HSI  = 2,
  STM32_LPTIM_CLK_LSE  = 3,
  STM32_LPTIM_CLK_EXT  = 4
} stm32_lptim_clksrc_t;

typedef enum
{
  STM32_LPTIM_COUNT_CLOCK   = 0,
  STM32_LPTIM_COUNT_EXTTRIG = 1
} stm32_lptim_cntmode_t;

typedef enum
{
  STM32_LPTIM_CLKPOL_RISING  = 0,
  STM32_LPTIM_CLKPOL_FALLING = 1,
  STM32_LPTIM_CLKPOL_BOTH    = 2
} stm32_lptim_clkpol_t;

typedef enum
{
  STM32_LPTIM_CH_DISABLED  = 0,
  STM32_LPTIM_CH_CHINVALID = 0,
  STM32_LPTIM_CH_CH1       = 1,
  STM32_LPTIM_CH_CH2       = 2,
  STM32_LPTIM_CH_CH3       = 3,
  STM32_LPTIM_CH_MASK      = 15
} stm32_lptim_channel_t;

struct stm32_lptim_dev_s
{
  const struct stm32_lptim_ops_s *ops;
};

struct stm32_lptim_ops_s
{
  int  (*setmode)(struct stm32_lptim_dev_s *dev,
                  stm32_lptim_mode_t mode);
  int  (*setclock)(struct stm32_lptim_dev_s *dev, uint32_t freq);
  int  (*setchannel)(struct stm32_lptim_dev_s *dev,
                     stm32_lptim_channel_t channel, int enable);
  int  (*setclocksource)(struct stm32_lptim_dev_s *dev,
                         stm32_lptim_clksrc_t clksrc);
  int  (*setpolarity)(struct stm32_lptim_dev_s *dev,
                      stm32_lptim_clkpol_t polarity);
  uint32_t (*getcounter)(struct stm32_lptim_dev_s *dev);
  int  (*setcountmode)(struct stm32_lptim_dev_s *dev,
                       stm32_lptim_cntmode_t cntmode);
  void (*setperiod)(struct stm32_lptim_dev_s *dev, uint32_t period);
  uint32_t (*getperiod)(struct stm32_lptim_dev_s *dev);
  void (*setcfgr)(struct stm32_lptim_dev_s *dev, uint32_t regval);
  void (*setcompare)(struct stm32_lptim_dev_s *dev, uint32_t cmp);
  int  (*setinput)(struct stm32_lptim_dev_s *dev, uint32_t input,
                   uint32_t mux);
  void (*enable)(struct stm32_lptim_dev_s *dev, bool on);
  void (*start)(struct stm32_lptim_dev_s *dev, stm32_lptim_mode_t mode);
  void (*resetcounter)(struct stm32_lptim_dev_s *dev);
  void (*enablerar)(struct stm32_lptim_dev_s *dev, bool on);

  /* Low-power timer interrupts */

  int  (*setisr)(struct stm32_lptim_dev_s *dev, xcpt_t handler,
                 void *arg, int source);
  void (*enableint)(struct stm32_lptim_dev_s *dev, int source);
  void (*disableint)(struct stm32_lptim_dev_s *dev, int source);
  void (*ackint)(struct stm32_lptim_dev_s *dev, int source);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Power-up and reset low power timer and get its structure */

struct stm32_lptim_dev_s *stm32_lptim_init(int lptimer);

/* Power-down low power timer, mark it as unused */

int stm32_lptim_deinit(struct stm32_lptim_dev_s *dev);

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_COMMON_STM32_STM32_LPTIM_H */

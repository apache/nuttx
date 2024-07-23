/****************************************************************************
 * arch/x86_64/src/intel64/intel64_hpet.h
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

#ifndef __ARCH_X86_64_SRC_INTEL64_INTEL64_HPET_H
#define __ARCH_X86_64_SRC_INTEL64_INTEL64_HPET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/compiler.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers ******************************************************************/

#define INTEL64_TIM_ENABLE(d, e)              ((d)->ops->enable(d, e))
#define INTEL64_TIM_GETPERIOD(d)              ((d)->ops->getperiod(d))
#define INTEL64_TIM_GETCOUNTER(d)             ((d)->ops->getcounter(d))
#define INTEL64_TIM_SETCOUNTER(d, c)          ((d)->ops->setcounter(d, c))
#define INTEL64_TIM_SETCOMPARE(d, t, c)       ((d)->ops->setcompare(d, t, c))
#define INTEL64_TIM_GETCOMPARE(d, t)          ((d)->ops->getcompare(d, t))
#define INTEL64_TIM_GETINT(d, t)              ((d)->ops->getint(d, t))
#define INTEL64_TIM_ACKINT(d, t)              ((d)->ops->ackint(d, t))
#define INTEL64_TIM_SETISR(d, t, h, a, p)     ((d)->ops->setisr(d, t, h, a, p))
#define INTEL64_TIM_ENABLEINT(d, t)           ((d)->ops->enableint(d, t))
#define INTEL64_TIM_DISABLEINT(d, t)          ((d)->ops->disableint(d, t))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* TIM Device Structure */

struct intel64_tim_ops_s;
struct intel64_tim_dev_s
{
  struct intel64_tim_ops_s *ops;
};

/* TIM Operations */

struct intel64_tim_ops_s
{
  void     (*enable)(struct intel64_tim_dev_s *dev, bool en);
  uint32_t (*getperiod)(struct intel64_tim_dev_s *dev);
  uint64_t (*getcounter)(struct intel64_tim_dev_s *dev);
  void     (*setcounter)(struct intel64_tim_dev_s *dev, uint64_t count);
  void     (*setcompare)(struct intel64_tim_dev_s *dev, uint8_t timer,
                         uint64_t compare);
  uint64_t (*getcompare)(struct intel64_tim_dev_s *dev, uint8_t timer);
  uint64_t (*getint)(struct intel64_tim_dev_s *dev, uint8_t timer);
  void     (*ackint)(struct intel64_tim_dev_s *dev, uint8_t timer);
  int      (*setisr)(struct intel64_tim_dev_s *dev, uint8_t timer,
                     xcpt_t handler, void * arg, bool periodic);
  void     (*enableint)(struct intel64_tim_dev_s *dev, uint8_t tim);
  void     (*disableint)(struct intel64_tim_dev_s *dev, uint8_t tim);
};

/****************************************************************************
 * Inline Functions
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
 * Name: intel64_hpet_init
 *
 * Description:
 *   Initialize HPET timer with a given base address
 *
 ****************************************************************************/

struct intel64_tim_dev_s *intel64_hpet_init(uint64_t base);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_X86_64_SRC_INTEL64_INTEL64_HPET_H */

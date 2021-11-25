/****************************************************************************
 * arch/arm/src/stm32h7/stm32_lptim.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_LPTIM_H
#define __ARCH_ARM_SRC_STM32H7_STM32_LPTIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/stm32_lptim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

typedef enum
{
  STM32_LPTIM_MODE_CONTINUOUS       = 0x00,
  STM32_LPTIM_MODE_SINGLE           = 0x01
} stm32_lptim_start_mode_t;

/* LPTIM Device Structure */

struct stm32_lptim_dev_s
{
  const  struct stm32_lptim_ops_s *ops;
  uint32_t                 base;
};

/* LPTIM Operations */

struct stm32_lptim_ops_s
{
  void (*setcfgr)(FAR struct stm32_lptim_dev_s *dev, uint32_t regval);
  void (*setperiod)(FAR struct stm32_lptim_dev_s *dev, uint32_t period);
  void (*setcompare)(FAR struct stm32_lptim_dev_s *dev, uint32_t cmp);
  uint32_t (*getcounter)(FAR struct stm32_lptim_dev_s *dev);
  int  (*setinput)(FAR struct stm32_lptim_dev_s *dev, uint32_t input,
                   uint32_t mux);
  void (*enable)(FAR struct stm32_lptim_dev_s *dev, bool on);
  void (*start)(FAR struct stm32_lptim_dev_s *dev,
                stm32_lptim_start_mode_t mode);
  void (*resetcounter)(FAR struct stm32_lptim_dev_s *dev);
  void (*enablerar)(FAR struct stm32_lptim_dev_s *dev, bool on);

  /* Low-power timer interrupts */

  int  (*setisr)(FAR struct stm32_lptim_dev_s *dev, xcpt_t handler,
                 void *arg, int source);
  void (*enableint)(FAR struct stm32_lptim_dev_s *dev, int source);
  void (*disableint)(FAR struct stm32_lptim_dev_s *dev, int source);
  void (*ackint)(FAR struct stm32_lptim_dev_s *dev, int source);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Power-up low power timer and get its structure */

FAR struct stm32_lptim_dev_s *stm32_lptim_init(int lptimer);

/* Power-down low power timer, mark it as unused */

int stm32_lptim_deinit(FAR struct stm32_lptim_dev_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_LPTIM_H */

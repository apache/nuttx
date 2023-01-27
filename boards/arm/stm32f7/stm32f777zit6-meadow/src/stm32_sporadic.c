/****************************************************************************
 * boards/arm/stm32f7/stm32f777zit6-meadow/src/stm32_sporadic.c
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

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/sched.h>

#include "stm32_gpio.h"
#include "stm32f777zit6-meadow.h"

#ifdef CONFIG_SPORADIC_INSTRUMENTATION

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arch_sporadic_*
 *
 * Description:
 *   This configuration has been used for evaluating the NuttX sporadic
 *   scheduler.  This only makes sense when uses with the sporadic test
 *   which is a part of apps/examples/ostest.  If would make generate
 *   meaningful output in its current state if there were multiple sporadic
 *   threads
 *
 ****************************************************************************/

void arch_sporadic_initialize(void)
{
  stm32_configgpio(GPIO_SCHED_HIGHPRI);
  stm32_configgpio(GPIO_SCHED_RUNNING);
}

void arch_sporadic_start(struct tcb_s *tcb)
{
  stm32_gpiowrite(GPIO_SCHED_HIGHPRI, true);
}

void arch_sporadic_lowpriority(struct tcb_s *tcb)
{
  stm32_gpiowrite(GPIO_SCHED_HIGHPRI, false);
}

void arch_sporadic_suspend(struct tcb_s *tcb)
{
  stm32_gpiowrite(GPIO_SCHED_RUNNING, false);
}

void arch_sporadic_resume(struct tcb_s *tcb)
{
  stm32_gpiowrite(GPIO_SCHED_RUNNING, true);
}

#endif /* CONFIG_SPORADIC_INSTRUMENTATION */

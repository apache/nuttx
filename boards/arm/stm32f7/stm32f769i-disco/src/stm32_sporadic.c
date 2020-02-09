/****************************************************************************
 * boards/arm/stm32f7/stm32f769i-disco/src/stm32_sporadic.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/sched.h>

#include "stm32_gpio.h"
#include "stm32f769i-disco.h"

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

void arch_sporadic_start(FAR struct tcb_s *tcb)
{
  stm32_gpiowrite(GPIO_SCHED_HIGHPRI, true);
}

void arch_sporadic_lowpriority(FAR struct tcb_s *tcb)
{
  stm32_gpiowrite(GPIO_SCHED_HIGHPRI, false);
}

void arch_sporadic_suspend(FAR struct tcb_s *tcb)
{
  stm32_gpiowrite(GPIO_SCHED_RUNNING, false);
}

void arch_sporadic_resume(FAR struct tcb_s *tcb)
{
  stm32_gpiowrite(GPIO_SCHED_RUNNING, true);
}

#endif /* CONFIG_SPORADIC_INSTRUMENTATION */

/****************************************************************************
 * arch/risc-v/src/eic7700x/eic7700x_cpuclk.h
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

#ifndef __ARCH_RISCV_SRC_EIC7700X_EIC7700X_CPUCLK_H
#define __ARCH_RISCV_SRC_EIC7700X_EIC7700X_CPUCLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>
#include <stdint.h>

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
 * Name: eic7700x_cpuclk_measure
 *
 * Description:
 *   Measure the core clock against the crystal-derived time counter.
 *
 *   This is a measurement, not a register read: it counts real cycles for
 *   ten milliseconds of real time and divides.  It exists because the
 *   documentation cannot be trusted about which PLL output the cores run
 *   from, and it spins for its whole sample window, so it is a tool for
 *   boot and for moments that can afford ten quiet milliseconds, not
 *   for anything periodic.
 *
 * Returned Value:
 *   The core clock in Hz as measured on the calling hart.
 *
 ****************************************************************************/

uint32_t eic7700x_cpuclk_measure(void);

/****************************************************************************
 * Name: eic7700x_cpuclk_rates
 *
 * Description:
 *   The speeds the cores may be asked for: the vendor's validated
 *   operating points, ascending, all at the same core voltage.
 *
 * Input Parameters:
 *   count - Where to put how many there are.
 *
 * Returned Value:
 *   The table, which lives in flash and never changes.
 *
 ****************************************************************************/

const uint32_t *eic7700x_cpuclk_rates(size_t *count);

/****************************************************************************
 * Name: eic7700x_cpuclk_setrate
 *
 * Description:
 *   Move the four cores to another speed.
 *
 *   The whole transition is owned here: the cores are parked on a slow
 *   clock, the PLL is reprogrammed and watched until it locks, the bus
 *   ratio rule is applied, and only then do the cores return.  Takes a
 *   few hundred microseconds, sleeps on a mutex, and must not be called
 *   from an interrupt.
 *
 * Input Parameters:
 *   hz - One of the rates from eic7700x_cpuclk_rates().
 *
 * Returned Value:
 *   Zero on success.  -EINVAL for a rate not in the table.  -ETIMEDOUT
 *   if the PLL would not lock, in which case the cores are left parked
 *   at four hundred megahertz: slow, but alive and debuggable.
 *
 ****************************************************************************/

int eic7700x_cpuclk_setrate(uint32_t hz);

/****************************************************************************
 * Name: eic7700x_cpuclk_initialize
 *
 * Description:
 *   Measure the cores, say so in the log, and move them to the configured
 *   speed if that is not where they already are.
 *
 ****************************************************************************/

void eic7700x_cpuclk_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_EIC7700X_EIC7700X_CPUCLK_H */

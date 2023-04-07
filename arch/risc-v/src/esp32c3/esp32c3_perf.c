/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_perf.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this args for additional information regarding copyright ownership.  The
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
#include <nuttx/clock.h>

#include <stdint.h>
#include <time.h>

#include "esp32c3_attr.h"
#include "riscv_internal.h"
#include "hardware/esp32c3_system.h"
#include "esp32c3_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NSEC_PER_CYCLE (1000 / CONFIG_ESP32C3_CPU_FREQ_MHZ)
#define CYCLE_PER_SEC  (USEC_PER_SEC * CONFIG_ESP32C3_CPU_FREQ_MHZ)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_perf_init
 ****************************************************************************/

void up_perf_init(void *arg)
{
  WRITE_CSR(CSR_PCER_MACHINE, 0x1);
  WRITE_CSR(CSR_PCMR_MACHINE, 0x1);
}

/****************************************************************************
 * Name: up_perf_gettime
 ****************************************************************************/

unsigned long IRAM_ATTR up_perf_gettime(void)
{
  return READ_CSR(CSR_PCCR_MACHINE);
}

/****************************************************************************
 * Name: up_perf_getfreq
 ****************************************************************************/

unsigned long up_perf_getfreq(void)
{
  return CYCLE_PER_SEC;
}

/****************************************************************************
 * Name: up_perf_convert
 ****************************************************************************/

void up_perf_convert(unsigned long elapsed, struct timespec *ts)
{
  ts->tv_sec  = elapsed / CYCLE_PER_SEC;
  elapsed    -= ts->tv_sec * CYCLE_PER_SEC;
  ts->tv_nsec = elapsed * NSEC_PER_CYCLE;
}

/****************************************************************************
 * arch/arm/src/rtl8720c/ameba_start.c
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
#include <nuttx/init.h>
#include "arm_internal.h"
#include "rtl8710c_irq.h"
#include <stdlib.h>

extern void ram_start(void);
extern void ameba_lto(void);
const hal_irq_api_t sys_irq_api =
{
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ARMV8M_STACKCHECK

/* We need to get r10 set before we can allow instrumentation calls */

void __start(void) __attribute__((no_instrument_function));
#endif
void __start(void)
{
#ifdef CONFIG_ARMV8M_STACKCHECK

  /* Set the stack limit before we attempt to call any functions */

  #define STACKSIZE_TEMP CONFIG_IDLETHREAD_STACKSIZE
  __asm__ volatile("sub r10, sp, %0" : : "r" (STACKSIZE_TEMP - 64) :);
#endif
  ram_start();
}

void mpu_init(void)
{
}

void promisc_deinit(void *padapter)
{
}

int promisc_recv_func(void *padapter, void *rframe)
{
  return 0;
}

int promisc_recv_lens_func(void *padapter, uint8_t *payload, uint8_t plen)
{
  return 0;
}

void app_start(void)
{
  __asm volatile("MSR msplim, %0" : : "r"(0));
  arm_earlyserialinit();
#ifdef CONFIG_MBEDTLS240_AMEBAZ_HARDWARE_CRYPTO
  extern int mbedtls_platform_set_calloc_free(
    void *(*calloc_func)(size_t, size_t), void (*free_func)(void *));
  extern int chip_platform_set_malloc_free(
    void *(*ssl_calloc)(unsigned int, unsigned int),
    void (*ssl_free)(void *));
  mbedtls_platform_set_calloc_free(calloc, free);
  chip_platform_set_malloc_free(calloc, free);
#endif
  nx_start();
  ameba_lto();
}


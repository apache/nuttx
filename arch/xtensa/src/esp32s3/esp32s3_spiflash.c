/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_spiflash.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>
#include <string.h>
#include <sys/types.h>
#include <inttypes.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>

#include "hardware/esp32s3_soc.h"

#include "esp32s3_irq.h"

#include "esp32s3_spiflash.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Structure holding SPI flash access critical sections management
 * functions.
 */

struct spiflash_guard_funcs_s
{
  void (*start)(void);      /* critical section start function */
  void (*end)(void);        /* critical section end function */
  void (*op_lock)(void);    /* flash access API lock function */
  void (*op_unlock)(void);  /* flash access API unlock function */

  /* checks flash write addresses */

  bool (*address_is_safe)(size_t addr, size_t size);

  void (*yield)(void);      /* yield to the OS during flash erase */
};

struct spiflash_cachestate_s
{
  uint32_t value;
  irqstate_t flags;
};

/****************************************************************************
 * Private Functions Declaration
 ****************************************************************************/

static void spiflash_start(void);
static void spiflash_end(void);

/****************************************************************************
 * Public Functions Declaration
 ****************************************************************************/

extern void spi_flash_guard_set(const struct spiflash_guard_funcs_s *funcs);
extern uint32_t cache_suspend_icache(void);
extern void cache_resume_icache(uint32_t val);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spiflash_guard_funcs_s g_spi_flash_guard_funcs =
{
  .start           = spiflash_start,
  .end             = spiflash_end,
  .op_lock         = NULL,
  .op_unlock       = NULL,
  .address_is_safe = NULL,
  .yield           = NULL,
};

struct spiflash_cachestate_s g_state;

/****************************************************************************
 * Name: spiflash_opstart
 *
 * Description:
 *   Prepare for an SPIFLASH operation.
 *
 ****************************************************************************/

static IRAM_ATTR void spiflash_start(void)
{
  g_state.flags = enter_critical_section();
  g_state.value = cache_suspend_icache() << 16;
}

/****************************************************************************
 * Name: spiflash_opdone
 *
 * Description:
 *   Undo all the steps of opstart.
 *
 ****************************************************************************/

static IRAM_ATTR void spiflash_end(void)
{
  cache_resume_icache(g_state.value >> 16);
  leave_critical_section(g_state.flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_spiflash_init
 *
 * Description:
 *   Initialize ESP32-S3 SPI flash driver.
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int esp32s3_spiflash_init(void)
{
  spi_flash_guard_set(&g_spi_flash_guard_funcs);

  return OK;
}

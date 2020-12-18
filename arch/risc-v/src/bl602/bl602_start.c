/****************************************************************************
 * arch/risc-v/src/bl602/bl602_init.c
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

#include <stdint.h>

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <arch/board/board.h>

#include "riscv_internal.h"
#include "chip.h"

#include "bl602_boot2.h"
#include "hardware/bl602_hbn.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#define showprogress(c) up_lowputc(c)
#else
#define showprogress(c)
#endif

#define PARTITION_BOOT2_RAM_ADDR_ACTIVE (0x42049C00)
#define PARTITION_HEADER_BOOT2_RAM_ADDR (0x42049C04)
#define PARTITION_BOOT2_FLASH_HEADER    (0x42049d14)
#define PARTITION_BOOT2_FLASH_CONFIG    (0x42049d18)
#define PARTITION_MAGIC                 (0x54504642)
#define PARTITION_FW_PART_NAME          "FW"
#define PARTITION_FW_PART_HEADER_SIZE   (0x1000)

/* TODO use header file from project */

#define FW_XIP_ADDRESS (0x23000000)

#define BL602_IDLESTACK_SIZE (CONFIG_IDLETHREAD_STACKSIZE & ~3)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* g_idle_topstack: _sbss is the start of the BSS region as defined by the
 * linker script. _ebss lies at the end of the BSS region. The idle task
 * stack starts at the end of BSS and is of size CONFIG_IDLETHREAD_STACKSIZE.
 * The IDLE thread is the thread that the system boots on and, eventually,
 * becomes the IDLE, do nothing task that runs only when there is nothing
 * else to run.  The heap continues from there until the end of memory.
 * g_idle_topstack is a read-only variable the provides this computed
 * address.
 */

static uint8_t idle_stack[BL602_IDLESTACK_SIZE];

/* Dont change the name of varaible, since we refer this
 * boot2_partition_table in linker script
 */

static struct
{
  uint8_t                        partition_active_idx;
  uint8_t                        pad[3];
  struct pt_table_stuff_config_s table;
} boot2_partition_table;

/****************************************************************************
 * Public Data
 ****************************************************************************/

uint32_t g_idle_topstack = (uintptr_t)idle_stack + BL602_IDLESTACK_SIZE;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern void bl602_lowsetup(void);
extern void exception_common(void);
extern void bl602_boardinitialize(void);

/****************************************************************************
 * Name: boot2_get_flash_addr
 ****************************************************************************/

uint32_t boot2_get_flash_addr(void)
{
  extern uint8_t __boot2_flash_cfg_src;

  return (uint32_t)(&__boot2_flash_cfg_src +
                    (sizeof(boot2_partition_table.table.pt_entries[0]) *
                     boot2_partition_table.table.pt_table.entry_cnt));
}

/****************************************************************************
 * Name: bfl_main
 ****************************************************************************/

void bfl_main(void)
{
  uint32_t tmp_val;

  /* set interrupt vector */

  asm volatile("csrw mtvec, %0" ::"r"((uintptr_t)exception_common + 2));

  /* Configure the UART so we can get debug output */

  bl602_lowsetup();

  /* HBN Config AON pad input and SMT */

  tmp_val = BL_RD_REG(HBN_BASE, HBN_IRQ_MODE);
  tmp_val = BL_SET_REG_BITS_VAL(tmp_val, HBN_REG_AON_PAD_IE_SMT, 1);
  BL_WR_REG(HBN_BASE, HBN_IRQ_MODE, tmp_val);

#ifdef USE_EARLYSERIALINIT
  up_earlyserialinit();
#endif

  /* Do board initialization */

  bl602_boardinitialize();

  /* Call nx_start() */

  nx_start();

  /* Shouldn't get here */

  while (1)
    ;
}


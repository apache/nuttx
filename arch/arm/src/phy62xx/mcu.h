/****************************************************************************
 * arch/arm/src/phy62xx/mcu.h
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
 *    Filename:       bus_dev.h
 *    Revised:
 *    Revision:
 *
 *    Description:    Describe the purpose and contents of the file.
 ****************************************************************************/

#ifndef _HAL_MCU_H
#define _HAL_MCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "types.h"
#include <stdint.h>
#include "rom_sym_def.h"
#include <arch/irq.h>

/* enum{
 *   MCU_UNDEF    = 0,
 *   MCU_PRIME_A1 = 1,
 *   MCU_PRIME_A2 = 2,
 *   MCU_BUMBEE_M0,
 *   MCU_BUMBEE_CK802
 * };
 */

#define MCU_UNDEF 0
#define MCU_PRIME_A1 1
#define MCU_PRIME_A2 2
#define MCU_BUMBEE_M0 3
#define MCU_BUMBEE_CK802 4

#define SRAM_BASE_ADDR          0x1fff0000
#define SRAM_END_ADDR           0x1fffffff

#define ROM_SRAM_JUMPTABLE      SRAM_BASE_ADDR
#define ROM_SRAM_GLOBALCFG      (ROM_SRAM_JUMPTABLE+0x400)
#define ROM_SRAM_JUMPTABLE_MIRROR   0x1fffd000
#define ROM_SRAM_GLOBALCFG_MIRROR   (ROM_SRAM_JUMPTABLE_MIRROR+0x400)

#define ROM_SRAM_HEAP           0x1fffe000
#define ROM_SRAM_HEAP_SIZE      (1024*8)
#define ROM_SRAM_DWC_BUF        0x1ffffc00

#define APP_SRAM_START_ADDR     0x1fff2000

#define MAXMEMHEAP 4096

#define HAL_ISER   *((volatile uint32_t *)(0xe000e100))
#define HAL_ICER   *((volatile uint32_t *)(0xe000e180))

/* subWriteReg: write value to register zone: bit[high:low] */

/* #define subWriteReg(addr,high,low,value)    write_reg(addr,read_reg(addr)&
 *           (~((((unsigned int)1<<((high)-(low)+1))-1)<<(low)))|
 *           ((unsigned int)(value)<<(low)))
 */

#define subWriteReg(addr,high,low,value) write_reg(addr, ((read_reg(addr)) & (~((((unsigned int)1<<((high)-(low)+1))-1)<<(low)))) | ((unsigned int)(value)<<(low)))

#define TIME_BASE               (0x003fffff) /* 24bit count shift 2 bit as 1us/bit */
#define TIME_DELTA(x,y)         ( (x>=y) ? x-y : TIME_BASE-y+x )

extern void drv_irq_init(void);
extern int drv_enable_irq(void);
extern int drv_disable_irq(void);

/* extern irqstate_t up_irq_save(void);
 * extern void up_irq_restore(irqstate_t flags);
 */

#define HAL_CRITICAL_SECTION_INIT()   drv_irq_init()

/* #define HAL_ENTER_CRITICAL_SECTION()  drv_disable_irq()
 * #define HAL_EXIT_CRITICAL_SECTION()   drv_enable_irq()
 */

#define _HAL_CS_ALLOC_()  uint32_t cs;
#define HAL_ENTER_CRITICAL_SECTION()  cs = up_irq_save();
#define HAL_EXIT_CRITICAL_SECTION()   up_irq_restore(cs)

#endif

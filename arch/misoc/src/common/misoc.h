/****************************************************************************
 * arch/misoc/src/common/misoc.h
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

#ifndef __ARCH_MISOC_SRC_COMMON_MISOC_H
#define __ARCH_MISOC_SRC_COMMON_MISOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Low-level register access */

#define getreg8(a)            (*(volatile uint8_t *)(a))
#define putreg8(v,a)          (*(volatile uint8_t *)(a) = (v))
#define getreg16(a)           (*(volatile uint16_t *)(a))
#define putreg16(v,a)         (*(volatile uint16_t *)(a) = (v))
#define getreg32(a)           (*(volatile uint32_t *)(a))
#define putreg32(v,a)         (*(volatile uint32_t *)(a) = (v))

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: flush_cpu_dcache
 *
 * Description:
 *  flush cpu cache Data cache
 *
 ****************************************************************************/

void flush_cpu_dcache(void);

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   misoc_earlyserialinit was called previously.
 *
 ****************************************************************************/

void misoc_serial_initialize(void);

/****************************************************************************
 * Name: up_net_initialize
 *
 * Description:
 *   Register Network
 *
 ****************************************************************************/

int misoc_net_initialize(int intf);

/****************************************************************************
 * Name: misoc_puts
 *
 * Description:
 *   This is a low-level helper function used to support debug.
 *
 ****************************************************************************/

void misoc_puts(const char *str);

/****************************************************************************
 * Name: misoc_lowputc
 *
 * Description:
 *   Low-level, blocking character output the serial console.
 *
 ****************************************************************************/

void misoc_lowputc(char ch);

/****************************************************************************
 * Name: misoc_lowputs
 *
 * Description:
 *   This is a low-level helper function used to support debug.
 *
 ****************************************************************************/

void misoc_lowputs(const char *str);

/****************************************************************************
 * Name: modifyreg[N]
 *
 * Description:
 *   Atomic modification of registers.
 *
 ****************************************************************************/

void modifyreg8(unsigned int addr, uint8_t clearbits, uint8_t setbits);
void modifyreg16(unsigned int addr, uint16_t clearbits, uint16_t setbits);
void modifyreg32(unsigned int addr, uint32_t clearbits, uint32_t setbits);

/****************************************************************************
 * Name: misoc_flush_dcache
 *
 * Description:
 *   Flush the data cache of the cpu
 *
 ****************************************************************************/

void misoc_flush_dcache(void);

/****************************************************************************
 * Name: misoc_flush_icache
 *
 * Description:
 *   Flush the instruction cache of the cpu
 *
 ****************************************************************************/

void misoc_flush_icache(void);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MISOC_SRC_COMMON_MISOC_H */

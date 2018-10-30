/************************************************************************************
 * arch/risc-v/src/gap8/gap8_uart.h
 * UART driver on uDMA subsystem for GAP8
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: hhuysqt <1020988872@qq.com>
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
 ************************************************************************************/

/************************************************************************************
 *  This UART IP has no flow control. So ioctl is limited.
 ************************************************************************************/

#ifndef _ARCH_RISCV_SRC_GAP8_UART_H
#define _ARCH_RISCV_SRC_GAP8_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include "gap8.h"
#include "gap8_udma.h"
#include "gap8_gpio.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define GAP8_NR_UART 1

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

void up_earlyserialinit(void);
void up_serialinit(void);
int up_putc(int ch);

#endif /* _ARCH_RISCV_SRC_GAP8_UART_H */

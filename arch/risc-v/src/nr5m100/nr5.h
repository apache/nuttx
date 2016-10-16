/************************************************************************************
 * arch/risc-v/src/nr5m100/nr5.h
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Authors: Uros Platise <uros.platise@isotel.eu>
 *            Gregory Nutt <gnutt@nuttx.org>
 *            Ken Pettit <pettitkd@gmail.com>
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

#ifndef __ARCH_RISCV_SRC_NR5M100_NR5_H
#define __ARCH_RISCV_SRC_NR5M100_NR5_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <arch/irq.h>
#include "up_internal.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Peripherals **********************************************************************/

#include "chip.h"
#include "nr5_csr.h"
#include "nr5_lowputc.h"
#include "nr5_timer.h"
#include "nr5_uart.h"

#endif /* __ARCH_RISCV_SRC_NR5_NR5_H */


/************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_start.h
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC17_START_H
#define __ARCH_ARM_SRC_LPC17XX_LPC17_START_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* g_idle_topstack: _sbss is the start of the BSS region as defined by the
 * linker script. _ebss lies at the end of the BSS region. The idle task
 * stack starts at the end of BSS and is of size CONFIG_IDLETHREAD_STACKSIZE.
 * The IDLE thread is the thread that the system boots on and, eventually,
 * becomes the IDLE, do nothing task that runs only when there is nothing
 * else to run.  The heap continues from there until the end of memory.
 * g_idle_topstack is a read-only variable the provides this computed
 * address.
 */

extern const uintptr_t g_idle_topstack;

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: lpc17_boardinitialize
 *
 * Description:
 *   All LPC17xx architectures must provide the following entry point.  This entry
 *   point is called early in the initialization -- after clocking and memory have
 *   been configured but before caches have been enabled and before any devices have
 *   been initialized.
 *
 ************************************************************************************/

void lpc17_boardinitialize(void);

#endif /* __ARCH_ARM_SRC_LPC17XX_LPC17_START_H */

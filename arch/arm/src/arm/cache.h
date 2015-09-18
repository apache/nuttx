/****************************************************************************
 * arch/arm/src/arm/cache.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Several of these cache operations come from Atmel sample code with
 * modifications for better integration with NuttX.  The Atmel sample code
 * has a BSD compatibile license that requires this copyright notice:
 *
 *   Copyright (c) 2008, Atmel Corporation
 *
 * [Actually, I think that all of the Atmel functions are commented out now]
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
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_ARM_CACHE_H
#define __ARCH_ARM_SRC_ARM_CACHE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Defintiions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void         cp15_flush_idcache(uint32_t start, uint32_t end);
#if 0 /* Not used */
void         cp15_invalidate_idcache(void);
void         cp15_invalidate_icache(void);
#endif
void         cp15_invalidate_dcache(uint32_t start, uint32_t end);
#if 0 /* Not used */
void         cp15_invalidate_dcache_all(void);
void         cp15_prefetch_icacheline(unsigned int value);
void         cp15_testcleaninvalidate_dcache(void);
void         cp15_drain_writebuffer(void);

unsigned int cp15_read_dcachelockdown(void);
void         cp15_write_dcachelockdown(unsigned int value);
unsigned int cp15_read_icachelockdown(void);
void         cp15_write_icachelockdown(unsigned int value);
#endif

#endif /* __ARCH_ARM_SRC_ARM_CACHE_H */


/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_scufifo.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_SCUFIFO_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_SCUFIFO_H

/****************************************************************************
 * include files
 ****************************************************************************/

#define FIFOMEM_INVALID 0xffff

/****************************************************************************
 * Name: scufifo_initialize
 *
 * Description:
 *   Initialize SCU FIFO memory management
 *
 ****************************************************************************/

void scufifo_initialize(void);

/****************************************************************************
 * Name: scufifo_memalloc
 *
 * Description:
 *   Allocate SCU FIFO memory
 *
 * Input Parameters:
 *   size - Request memory size
 *
 * Returned Value:
 *   Allocated FIFO memory start offset. If error, return FIFOMEM_INVALID.
 *
 ****************************************************************************/

uint16_t scufifo_memalloc(uint16_t size);

/****************************************************************************
 * Name: scufifo_memfree
 *
 * Description:
 *   Free allocated SCU FIFO memory
 *
 * Input Parameters:
 *   start - Start offset of FIFO memory
 *
 ****************************************************************************/

void scufifo_memfree(uint16_t start);

#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_SCUFIFO_H */

/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_cpufifo.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_CPUFIFO_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_CPUFIFO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD56_FIF_PUSH_FULL    (CXD56_CPUFIFO_BASE + 0x00)
#define CXD56_FIF_PUSH_WRD0    (CXD56_CPUFIFO_BASE + 0x04)
#define CXD56_FIF_PUSH_WRD1    (CXD56_CPUFIFO_BASE + 0x08)
#define CXD56_FIF_PUSH_CMP     (CXD56_CPUFIFO_BASE + 0x0c)
#define CXD56_FIF_PULL_EMP     (CXD56_CPUFIFO_BASE + 0x10)
#define CXD56_FIF_PULL_WRD0    (CXD56_CPUFIFO_BASE + 0x14)
#define CXD56_FIF_PULL_WRD1    (CXD56_CPUFIFO_BASE + 0x18)
#define CXD56_FIF_PULL_CMP     (CXD56_CPUFIFO_BASE + 0x1c)

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_CPUFIFO_H */

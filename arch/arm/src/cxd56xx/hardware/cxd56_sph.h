/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_sph.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_SPH_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_SPH_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include "chip.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#define CXD56_SPH_REQ(n)    (CXD56_SPH_BASE + ((n) * 16) + 0)
#define CXD56_SPH_STS(n)    (CXD56_SPH_BASE + ((n) * 16) + 4)
#define CXD56_SPH_RESET(n)  (CXD56_SPH_BASE + ((n) * 4))

/* Hardware semaphore request */

#define REQ_UNLOCK              0
#define REQ_LOCK                1
#define REQ_RESERVE             2
#define REQ_INTRCLR             3

/* Hardware semaphore status [3:0] */

#define STATE_IDLE              0
#define STATE_LOCKED            1
#define STATE_LOCKEDANDRESERVED 2

#define STS_STATE(sts)  ((sts) & 0xf)
#define LOCK_OWNER(sts) (((sts) >> 16) & 0x1f)
#define RESV_OWNER(sts) (((sts) >> 24) & 0x1f)

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_SPH_H */

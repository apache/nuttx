/****************************************************************************
 * arch/arm/src/imxrt/imxrt_enc.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author:  Nicholas Chin <nicholaschin1995@gmail.com>
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_ENC_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_ENC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/imxrt_enc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define QEIOC_POSDIFF       _QEIOC(QE_IMXRT_FIRST)
#define QEIOC_REVOLUTION    _QEIOC(QE_IMXRT_FIRST + 1)
#define QEIOC_RECONFIG      _QEIOC(QE_IMXRT_FIRST + 2)
#define QEIOC_INITTO        _QEIOC(QE_IMXRT_FIRST + 3)
#define QEIOC_RESETAT       _QEIOC(QE_IMXRT_FIRST + 4)
#define QEIOC_RESETATMAX    _QEIOC(QE_IMXRT_FIRST + 5)
#define QEIOC_TEST_GEN      _QEIOC(QE_IMXRT_FIRST + 6)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_qeinitialize
 *
 * Description:
 *   Initialize a quadrature encoder interface.  This function must be called from
 *   board-specific logic..
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qe0"
 *   enc     - The encoder peripheral to use.  'enc' must be an element of {1,2,3,4}
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

int imxrt_qeinitialize(FAR const char *devpath, int enc);

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_ENC_H */

/****************************************************************************
 * arch/arm/src/tiva/tiva_qencoder.h
 *
 *   Copyright (C) 2016 Young Mu. All rights reserved.
 *   Author: Young Mu <young.mu@aliyun.com>
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

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_QENCODER_H
#define __ARCH_ARM_SRC_TIVA_TIVA_QENCODER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sensors/qencoder.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define QEIOC_DIRECTION     _QEIOC(QE_TIVA_FIRST)
#define QEIOC_VELOCITY      _QEIOC(QE_TIVA_FIRST+1)
#define QEIOC_RESETATPPR    _QEIOC(QE_TIVA_FIRST+2)
#define QEIOC_RESETATMAXPOS _QEIOC(QE_TIVA_FIRST+3)
#define QEIOC_RESETATINDEX  _QEIOC(QE_TIVA_FIRST+4)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct qe_lowerhalf_s *tiva_qei_initialize(int id);

#endif /* __ARCH_ARM_SRC_TIVA_TIVA_QENCODER_H */

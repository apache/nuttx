/****************************************************************************
 * arch/arm/src/lc823450/lc823450_pwm.h
 *
 *   Copyright 2014,2015,2017 Sony Video & Sound Products Inc.
 *   Author: Nobutaka Toyoshima <Nobutaka.Toyoshima@jp.sony.com>
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_PWM_H
#define __ARCH_ARM_SRC_LC823450_LC823450_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/drivers/pwm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Addresses *******************************************************/

#define LC823450_MTM0_REGBASE   0x40043000
#define LC823450_MTM1_REGBASE   0x40044000

#define LC823450_MTM_OPR        0x00
#define LC823450_MTM_SYNC       0x04
#define LC823450_MTM_PWM        0x08
#define LC823450_MTM_FCTL       0x0c

#define LC823450_MTM_0CTL       0x40
#define LC823450_MTM_0IOCL      0x44
#define LC823450_MTM_0STS       0x4C
#define LC823450_MTM_0CNT       0x50
#define LC823450_MTM_0A         0x54
#define LC823450_MTM_0B         0x58
#define LC823450_MTM_0SOL       0x5C
#define LC823450_MTM_0BA        0x60
#define LC823450_MTM_0BB        0x64
#define LC823450_MTM_0PSCL      0x68
#define LC823450_MTM_0TIER      0x6C
#define LC823450_MTM_0TISR      0x70

#define LC823450_MTM_1CTL       0x80
#define LC823450_MTM_1IOCL      0x84
#define LC823450_MTM_1STS       0x8C
#define LC823450_MTM_1CNT       0x90
#define LC823450_MTM_1A         0x94
#define LC823450_MTM_1B         0x98
#define LC823450_MTM_1SOL       0x9C
#define LC823450_MTM_1BA        0xA0
#define LC823450_MTM_1BB        0xA4
#define LC823450_MTM_1PSCL      0xA8
#define LC823450_MTM_1TIER      0xAC
#define LC823450_MTM_1TISR      0xB0

/* PWM Identifier ***********************************************************/

#define LC823450_PWMTIMER0_CH0  0
#define LC823450_PWMTIMER0_CH1  1
#define LC823450_PWMTIMER1_CH0  2
#define LC823450_PWMTIMER1_CH1  3

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

EXTERN FAR struct pwm_lowerhalf_s *lc823450_pwminitialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_LC823450_LC823450_PWM_H */

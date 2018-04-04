/****************************************************************************
 * arch/arm/src/lc823450/lc823450_intc.h
 *
 *   Copyright 2014,2017 Sony Video & Sound Products Inc.
 *   Author: Nobutaka Toyoshima <Nobutaka.Toyoshima@jp.sony.com>
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
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

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_INTC_H
#define __ARCH_ARM_SRC_LC823450_LC823450_INTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LC823450_INTC_REGBASE   0x40003000

#define IPIREG          (LC823450_INTC_REGBASE + 0x000)
#define   IPIREG_INTISR0_0      (0x1 << 0)
#define   IPIREG_INTISR0_1      (0x1 << 1)
#define   IPIREG_INTISR0_2      (0x1 << 2)
#define   IPIREG_INTISR0_3      (0x1 << 3)
#define   IPIREG_INTISR1_0      (0x1 << 8)
#define   IPIREG_INTISR1_1      (0x1 << 9)
#define   IPIREG_INTISR1_2      (0x1 << 10)
#define   IPIREG_INTISR1_3      (0x1 << 11)
#define IPICLR          (LC823450_INTC_REGBASE + 0x004)
#define   IPICLR_INTISR0_CLR_0  (0x1 << 0)
#define   IPICLR_INTISR0_CLR_1  (0x1 << 1)
#define   IPICLR_INTISR0_CLR_2  (0x1 << 2)
#define   IPICLR_INTISR0_CLR_3  (0x1 << 3)
#define   IPICLR_INTISR1_CLR_0  (0x1 << 8)
#define   IPICLR_INTISR1_CLR_1  (0x1 << 9)
#define   IPICLR_INTISR1_CLR_2  (0x1 << 10)
#define   IPICLR_INTISR1_CLR_3  (0x1 << 11)

#define EXTINTn_BASE    (LC823450_INTC_REGBASE + 0x400)
#define EXTINTnS_BASE   (LC823450_INTC_REGBASE + 0x418)
#define EXTINTnM_BASE   (LC823450_INTC_REGBASE + 0x430)
#define EXTINTnC0_BASE  (LC823450_INTC_REGBASE + 0x448)
#define EXTINTnC1_BASE  (LC823450_INTC_REGBASE + 0x460)
#define EXTINTnCND_BASE (LC823450_INTC_REGBASE + 0x478)
#define EXTINTnCLR_BASE (LC823450_INTC_REGBASE + 0x490)
#define EXTINTnFEN_BASE (LC823450_INTC_REGBASE + 0x4A8)
#define EXTINTnSET_BASE (LC823450_INTC_REGBASE + 0x4C0)

#define INTC_REG(base,port)  ((base) + 4 * (port))

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LC823450_LC823450_INTC_H */

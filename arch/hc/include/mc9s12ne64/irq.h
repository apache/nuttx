/************************************************************************************
 * arch/hc/include/mc9s12ne64/irq.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_HC_INCLUDE_MC9S12NE64_IRQ_H
#define __ARCH_HC_INCLUDE_MC9S12NE64_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* IRQ Numbers */

#define HCS12_IRQ_VRESET       0 /* fffe: External reset, power on reset orlow voltage reset */
#define HCS12_IRQ_VCLKMON      1 /* fffc: Clock monitor fail reset */
#define HCS12_IRQ_VCOP         2 /* fffa: COP failure reset*/
#define HCS12_IRQ_VTRAP        3 /* fff8: Unimplemented instruction trap */
#define HCS12_IRQ_VSWI         4 /* fff6: SWI */
#define HCS12_IRQ_VXIRQ        5 /* fff4: XIRQ */
#define HCS12_IRQ_VIRQ         6 /* fff2: IRQ */
#define HCS12_IRQ_VRTI         7 /* fff0: Real-time interrupt */
                                 /* ffe8-ffef: Reserved */
#define HCS12_IRQ_VTIMCH4      8 /* ffe6: Standard timer channel 4 */
#define HCS12_IRQ_VTIMCH5      9 /* ffe4: Standard timer channel 5 */
#define HCS12_IRQ_VTIMCH6     10 /* ffe2: Standard timer channel 6 */
#define HCS12_IRQ_VTIMCH7     11 /* ffe0: Standard timer channel 7 */
#define HCS12_IRQ_VTIMOVF     12 /* ffde: Standard timer overflow */
#define HCS12_IRQ_VTIMPAOVF   13 /* ffdc: Pulse accumulator overflow */
#define HCS12_IRQ_VTIMPAIE    14 /* ffda: Pulse accumulator input edge */
#define HCS12_IRQ_VSPI        15 /* ffd8: SPI */
#define HCS12_IRQ_VSCI0       16 /* ffd6: SCI0 */
#define HCS12_IRQ_VSCI1       17 /* ffd4: SCI1 */
#define HCS12_IRQ_VATD        18 /* ffd2: ATD */
                                 /* ffd0: Reserved */
#define HCS12_IRQ_VPORTJ      19 /* ffce: Port J */
#define HCS12_IRQ_VPORTH      20 /* ffcc: Port H */
#define HCS12_IRQ_VPORTG      21 /* ffca: Port G */
                                 /* ffc8: Reserved */
#define HCS12_IRQ_VCRGPLLLCK  22 /* ffc6: CRG PLL lock */
#define HCS12_IRQ_VCRGSCM     23 /* ffc4: CRG self clock mode */
                                 /* ffc2: Reserved */
#define HCS12_IRQ_VIIC        24 /* ffc0: IIC bus */
                                 /* ffba-ffbf: Reserved */
#define HCS12_IRQ_VFLASH      25 /* ffb8: FLASH */
#define HCS12_IRQ_VEPHY       26 /* ffb6: EPHY interrupt */
#define HCS12_IRQ_VEMACCRXBAC 27 /* ffb4: EMAC receive buffer A complete */
#define HCS12_IRQ_VEMACCRXBBC 28 /* ffb2: EMAC receive buffer B complete */
#define HCS12_IRQ_VEMACTXC    29 /* ffb0: EMAC frame transmission complete */
#define HCS12_IRQ_VEMACRXFC   30 /* ffae: EMAC receive flow control */
#define HCS12_IRQ_VEMACMII    31 /* ffac: EMAC MII management transfer complete */
#define HCS12_IRQ_VEMACRXERR  32 /* ffaa: EMAC receive error */
#define HCS12_IRQ_VEMACRXBAO  33 /* ffa8: EMAC receive buffer A overrun */
#define HCS12_IRQ_VEMACRXBBO  34 /* ffa6: EMAC receive buffer B overrun */
#define HCS12_IRQ_VEMACBRXERR 35 /* ffa4: EMAC babbling receive error */
#define HCS12_IRQ_VEMACLC     36 /* ffa2: EMAC late collision */
#define HCS12_IRQ_VEMACEC     37 /* ffa0: EMAC excessive collision */
                                 /* ff80-ff9f: Reserved */
#define HCS12_IRQ_VILLEGAL    38 /* Any reserved vector */
#define NR_IRQS               39

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_HC_INCLUDE_MC9S12NE64_IRQ_H */

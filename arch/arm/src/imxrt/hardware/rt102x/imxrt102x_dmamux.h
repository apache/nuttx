/*****************************************************************************
 * arch/arm/src/imxrt/hardware/rt102x/imxrt102x_dmamux.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Dave Marples <dave@marples.net>
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
 *****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT102X_DMAMUX_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT102X_DMAMUX_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

/*****************************************************************************
 * Preprocessor Definitions
 *****************************************************************************/

/* Peripheral DMA request channels */

#define IMXRT_DMACHAN_FLEXIO1_01       0  /* FlexIO1 DMA 0/1, Async DMA 0/1 */
#define IMXRT_DMACHAN_FLEXIO1_45       1  /* FlexIO1 DMA 4/5, Async DMA 4/5 */
#define IMXRT_DMACHAN_LPUART1_TX       2  /* LPUART1 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART1_RX       3  /* LPUART1 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART3_TX       4  /* LPUART3 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART3_RX       5  /* LPUART3 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART5_TX       6  /* LPUART5 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART5_RX       7  /* LPUART5 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART7_TX       8  /* LPUART7 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART7_RX       9  /* LPUART7 RX FIFO DMA / Async DMA */

#define IMXRT_DMACHAN_LPSPI1_RX       13  /* LPSPI1 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPSPI1_TX       14  /* LPSPI1 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPSPI3_RX       15  /* LPSPI3 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPSPI3_TX       16  /* LPSPI3 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPI2C1          17  /* LPI2C1 Master/Slave RX/TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPI2C3          18  /* LPI2C3 Master/Slave RX/TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_SAI1_RX         19  /* SAI1 RX FIFO DMA */
#define IMXRT_DMACHAN_SAI1_TX         20  /* SAI1 TX FIFO DMA */
#define IMXRT_DMACHAN_SAI2_RX         21  /* SAI2 RX FIFO DMA */
#define IMXRT_DMACHAN_SAI2_TX         22  /* SAI2 TX FIFO DMA */
#define IMXRT_DMACHAN_ADC_ETC         23  /* ADC ETC DMA */
#define IMXRT_DMACHAN_ADC1            24  /* ADC1 DMA */
#define IMXRT_DMACHAN_ACMP1           25  /* ACMP1 DMA */
#define IMXRT_DMACHAN_ACMP3           26  /* ACMP3 DMA */
#define IMXRT_DMACHAN_FLEXSPI_RX      28  /* FlexSPI RX FIFO DMA */
#define IMXRT_DMACHAN_FLEXSPI_TX      29  /* FlexSPI TX FIFO DMA */
#define IMXRT_DMACHAN_XBAR1_0         30  /* XBAR1 DMA 0 */
#define IMXRT_DMACHAN_XBAR1_1         31  /* XBAR1 DMA 1 */
#define IMXRT_DMACHAN_FLEXPWM1_RX0    32  /* FlexPWM1 RX sub-module0 capture */
#define IMXRT_DMACHAN_FLEXPWM1_RX1    33  /* FlexPWM1 RX sub-module1 capture */
#define IMXRT_DMACHAN_FLEXPWM1_RX2    34  /* FlexPWM1 RX sub-module2 capture */
#define IMXRT_DMACHAN_FLEXPWM1_RX3    35  /* FlexPWM1 RX sub-module3 capture */
#define IMXRT_DMACHAN_FLEXPWM1_TX0    36  /* FlexPWM1 TX sub-module0 value */
#define IMXRT_DMACHAN_FLEXPWM1_TX1    37  /* FlexPWM1 TX sub-module1 value */
#define IMXRT_DMACHAN_FLEXPWM1_TX2    38  /* FlexPWM1 TX sub-module2 value */
#define IMXRT_DMACHAN_FLEXPWM1_TX3    39  /* FlexPWM1 TX sub-module3 value */
#define IMXRT_DMACHAN_QTIMER1_RX0     48  /* QTimer1 RX capture timer 0 */
#define IMXRT_DMACHAN_QTIMER1_RX1     49  /* QTimer1 RX capture timer 1 */
#define IMXRT_DMACHAN_QTIMER1_RX2     50  /* QTimer1 RX capture timer 2 */
#define IMXRT_DMACHAN_QTIMER1_RX3     51  /* QTimer1 RX capture timer 3 */
#define IMXRT_DMACHAN_QTIMER1_TX0     52  /* QTimer1 TX cmpld1 timer 0 / cmld2 timer 1 */
#define IMXRT_DMACHAN_QTIMER1_TX1     53  /* QTimer1 TX cmpld1 timer 1 / cmld2 timer 0 */
#define IMXRT_DMACHAN_QTIMER1_TX2     54  /* QTimer1 TX cmpld1 timer 2 / cmld2 timer 3 */
#define IMXRT_DMACHAN_QTIMER1_TX3     55  /* QTimer1 TX cmpld1 timer 3 / cmld2 timer 2 */
#define IMXRT_DMACHAN_FLEXIO1_23      64  /* FlexIO1 DMA 2 / Async DMA 2 / DMA 3 / Async DMA 3 */
#define IMXRT_DMACHAN_FLEXIO1_67      65  /* FlexIO1 DMA 6 / Async DMA 6 / DMA 7 / Async DMA 7 */
#define IMXRT_DMACHAN_LPUART2_TX      66  /* LPUART2 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART2_RX      67  /* LPUART2 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART4_TX      68  /* LPUART4 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART4_RX      69  /* LPUART4 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART6_TX      70  /* LPUART6 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART6_RX      71  /* LPUART6 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART8_TX      72  /* LPUART8 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART8_RX      73  /* LPUART8 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPSPI2_RX       77  /* LPSPI2 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPSPI2_TX       78  /* LPSPI2 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPSPI4_RX       79  /* LPSPI4 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPSPI4_TX       80  /* LPSPI4 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPI2C2          81  /* LPI2C2 Master/Slave RX/TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPI2C4          82  /* LPI2C4 Master/Slave RX/TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_SAI3_RX         83  /* SAI3 RX FIFO DMA */
#define IMXRT_DMACHAN_SAI3_TX         84  /* SAI3 RX FIFO DMA */
#define IMXRT_DMACHAN_SPDIF_RX        85  /* SPDIF RX DMA */
#define IMXRT_DMACHAN_SPDIF_TX        86  /* SPDIF TX DMA */
#define IMXRT_DMACHAN_ADC2            88  /* ADC2 DMA */
#define IMXRT_DMACHAN_ACMP2           89  /* ACMP2 DMA */
#define IMXRT_DMACHAN_ACMP4           90  /* ACMP4 DMA */
#define IMXRT_DMACHAN_ENET_0          92  /* ENET Timer DMA 0 */
#define IMXRT_DMACHAN_ENET_1          93  /* ENET Timer DMA 1 */
#define IMXRT_DMACHAN_XBAR1_2         94  /* XBAR1 DMA 2 */
#define IMXRT_DMACHAN_XBAR1_3         95  /* XBAR1 DMA 3 */
#define IMXRT_DMACHAN_FLEXPWM2_RX0    96  /* FlexPWM2 RX sub-module0 capture */
#define IMXRT_DMACHAN_FLEXPWM2_RX1    97  /* FlexPWM2 RX sub-module1 capture */
#define IMXRT_DMACHAN_FLEXPWM2_RX2    98  /* FlexPWM2 RX sub-module2 capture */
#define IMXRT_DMACHAN_FLEXPWM2_RX3    99  /* FlexPWM2 RX sub-module3 capture */
#define IMXRT_DMACHAN_FLEXPWM2_TX0   100  /* FlexPWM2 TX sub-module0 value */
#define IMXRT_DMACHAN_FLEXPWM2_TX1   101  /* FlexPWM2 TX sub-module1 value */
#define IMXRT_DMACHAN_FLEXPWM2_TX2   102  /* FlexPWM2 TX sub-module2 value */
#define IMXRT_DMACHAN_FLEXPWM2_TX3   103  /* FlexPWM2 TX sub-module3 value */
#define IMXRT_DMACHAN_QTIMER2_RX0    112  /* QTimer2 RX capture timer 0 */
#define IMXRT_DMACHAN_QTIMER2_RX1    113  /* QTimer2 RX capture timer 1 */
#define IMXRT_DMACHAN_QTIMER2_RX2    114  /* QTimer2 RX capture timer 2 */
#define IMXRT_DMACHAN_QTIMER2_RX3    115  /* QTimer2 RX capture timer 3 */
#define IMXRT_DMACHAN_QTIMER2_TX0    116  /* QTimer2 TX cmpld1 timer 0 / cmld2 timer 1 */
#define IMXRT_DMACHAN_QTIMER2_TX1    117  /* QTimer2 TX cmpld1 timer 1 / cmld2 timer 0 */
#define IMXRT_DMACHAN_QTIMER2_TX2    118  /* QTimer2 TX cmpld1 timer 2 / cmld2 timer 3 */
#define IMXRT_DMACHAN_QTIMER2_TX3    119  /* QTimer2 TX cmpld1 timer 3 / cmld2 timer 2 */

#define IMXRT_DMA_NCHANNELS          128  /* Includes reserved channels */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT102X_DMAMUX_H */

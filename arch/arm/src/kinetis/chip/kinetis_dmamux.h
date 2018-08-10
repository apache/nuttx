/********************************************************************************************
 * arch/arm/src/kinetis/chip/kinetis_dmamux.h
 *
 *   Copyright (C) 2011, 2016, 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Jan Okle <jan@leitwert.ch>
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_DMAMUX_H
#define __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_DMAMUX_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register Offsets *************************************************************************/

#if KINETIS_DMAMUX_HAS_MONOTONIC_CHCFG == 0

  /* Channel n Configuration Register, 3,2,1,0,7,6,5,4 ..KINETIS_NDMACH-1 */

#  define KINETIS_DMAMUX_CHCFG_OFFSET(n)  (n - (n % 4)) + (3 - (n % 4))
#elif KINETIS_DMAMUX_HAS_MONOTONIC_CHCFG == 1

  /* Channel n Configuration Register, n=0..KINETIS_NDMACH-1 */

#  define KINETIS_DMAMUX_CHCFG_OFFSET(n)  (n)
#endif

/* Register Addresses ***********************************************************************/

#define KINETIS_DMAMUX_CHCFG(n)         (KINETIS_DMAMUX0_BASE+KINETIS_DMAMUX_CHCFG_OFFSET(n))

/* Register Bit Definitions *****************************************************************/
/* Channel n Configuration Register */

#define DMAMUX_CHCFG_SOURCE_SHIFT       (0)       /* Bits 0-5: DMA Channel Source (slot) */
#define DMAMUX_CHCFG_SOURCE_MASK        (63 << DMAMUX_CHCFG_SOURCE_SHIFT)
#define DMAMUX_CHCFG_TRIG               (1 << 6)  /* Bit 6:  DMA Channel Trigger Enable */
#define DMAMUX_CHCFG_ENBL               (1 << 7)  /* Bit 7:  DMA Channel Enable */

/* DMA Request sources*****************************************************************************/

#ifdef KINETIS_K60

#  define KINETIS_DMA_REQUEST_SRC_UART0_RX           2
#  define KINETIS_DMA_REQUEST_SRC_UART0_TX           3
#  define KINETIS_DMA_REQUEST_SRC_UART1_RX           4
#  define KINETIS_DMA_REQUEST_SRC_UART1_TX           5
#  define KINETIS_DMA_REQUEST_SRC_UART2_RX           6
#  define KINETIS_DMA_REQUEST_SRC_UART2_TX           7
#  define KINETIS_DMA_REQUEST_SRC_UART3_RX           8
#  define KINETIS_DMA_REQUEST_SRC_UART3_TX           9
#  define KINETIS_DMA_REQUEST_SRC_UART4_RX          10
#  define KINETIS_DMA_REQUEST_SRC_UART4_TX          11
#  define KINETIS_DMA_REQUEST_SRC_UART5_RX          12
#  define KINETIS_DMA_REQUEST_SRC_UART5_TX          13
#  define KINETIS_DMA_REQUEST_SRC_I2S0_RX           14
#  define KINETIS_DMA_REQUEST_SRC_I2S0_TX           15
#  define KINETIS_DMA_REQUEST_SRC_SPI0_RX           16
#  define KINETIS_DMA_REQUEST_SRC_SPI0_TX           17
#  define KINETIS_DMA_REQUEST_SRC_SPI1_RX           18
#  define KINETIS_DMA_REQUEST_SRC_SPI1_TX           19
#  define KINETIS_DMA_REQUEST_SRC_SPI2_RX           20
#  define KINETIS_DMA_REQUEST_SRC_SPI2_TX           21
#  define KINETIS_DMA_REQUEST_SRC_I2C0              22
#  define KINETIS_DMA_REQUEST_SRC_I2C1              23
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH0          24
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH1          25
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH2          26
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH3          27
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH4          28
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH5          29
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH6          30
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH7          31
#  define KINETIS_DMA_REQUEST_SRC_FTM1_CH0          32
#  define KINETIS_DMA_REQUEST_SRC_FTM1_CH1          33
#  define KINETIS_DMA_REQUEST_SRC_FTM2_CH0          34
#  define KINETIS_DMA_REQUEST_SRC_FTM2_CH1          35
#  define KINETIS_DMA_REQUEST_SRC_TIMER0            36
#  define KINETIS_DMA_REQUEST_SRC_TIMER1            37
#  define KINETIS_DMA_REQUEST_SRC_TIMER2            38
#  define KINETIS_DMA_REQUEST_SRC_TIMER3            39
#  define KINETIS_DMA_REQUEST_SRC_ADC0              40
#  define KINETIS_DMA_REQUEST_SRC_ADC1              41
#  define KINETIS_DMA_REQUEST_SRC_CMP0              42
#  define KINETIS_DMA_REQUEST_SRC_CMP1              43
#  define KINETIS_DMA_REQUEST_SRC_CMP2              44
#  define KINETIS_DMA_REQUEST_SRC_DAC0              45
#  define KINETIS_DMA_REQUEST_SRC_DAC1              46
#  define KINETIS_DMA_REQUEST_SRC_CMT               47
#  define KINETIS_DMA_REQUEST_SRC_PDB               48
#  define KINETIS_DMA_REQUEST_SRC_PCM_A             49
#  define KINETIS_DMA_REQUEST_SRC_PCM_B             50
#  define KINETIS_DMA_REQUEST_SRC_PCM_C             51
#  define KINETIS_DMA_REQUEST_SRC_PCM_D             52
#  define KINETIS_DMA_REQUEST_SRC_PCM_E             53

#endif /* KINETIS_K60 */

#ifdef KINETIS_K64

#  define KINETIS_DMA_REQUEST_SRC_UART0_RX           2
#  define KINETIS_DMA_REQUEST_SRC_UART0_TX           3
#  define KINETIS_DMA_REQUEST_SRC_UART1_RX           4
#  define KINETIS_DMA_REQUEST_SRC_UART1_TX           5
#  define KINETIS_DMA_REQUEST_SRC_UART2_RX           6
#  define KINETIS_DMA_REQUEST_SRC_UART2_TX           7
#  define KINETIS_DMA_REQUEST_SRC_UART3_RX           8
#  define KINETIS_DMA_REQUEST_SRC_UART3_TX           9
#  define KINETIS_DMA_REQUEST_SRC_UART4_RXTX        10
#  define KINETIS_DMA_REQUEST_SRC_UART5_RXTX        11
#  define KINETIS_DMA_REQUEST_SRC_I2S0_RX           12
#  define KINETIS_DMA_REQUEST_SRC_I2S0_TX           13
#  define KINETIS_DMA_REQUEST_SRC_SPI0_RX           14
#  define KINETIS_DMA_REQUEST_SRC_SPI0_TX           15
#  define KINETIS_DMA_REQUEST_SRC_SPI1_RXTX         16
#  define KINETIS_DMA_REQUEST_SRC_SPI2_RXTX         17
#  define KINETIS_DMA_REQUEST_SRC_I2C0              18
#  define KINETIS_DMA_REQUEST_SRC_I2C1__I2C2        19
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH0          20
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH1          21
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH2          22
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH3          23
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH4          24
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH5          25
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH6          26
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH7          27
#  define KINETIS_DMA_REQUEST_SRC_FTM1_CH0          28
#  define KINETIS_DMA_REQUEST_SRC_FTM1_CH1          29
#  define KINETIS_DMA_REQUEST_SRC_FTM2_CH0          30
#  define KINETIS_DMA_REQUEST_SRC_FTM2_CH1          31
#  define KINETIS_DMA_REQUEST_SRC_FTM3_CH0          32
#  define KINETIS_DMA_REQUEST_SRC_FTM3_CH1          33
#  define KINETIS_DMA_REQUEST_SRC_FTM3_CH2          34
#  define KINETIS_DMA_REQUEST_SRC_FTM3_CH3          35
#  define KINETIS_DMA_REQUEST_SRC_FTM3_CH4          36
#  define KINETIS_DMA_REQUEST_SRC_FTM3_CH5          37
#  define KINETIS_DMA_REQUEST_SRC_FTM3_CH6          38
#  define KINETIS_DMA_REQUEST_SRC_FTM3_CH7          39
#  define KINETIS_DMA_REQUEST_SRC_ADC0              40
#  define KINETIS_DMA_REQUEST_SRC_ADC1              41
#  define KINETIS_DMA_REQUEST_SRC_CMP0              42
#  define KINETIS_DMA_REQUEST_SRC_CMP1              43
#  define KINETIS_DMA_REQUEST_SRC_CMP2              44
#  define KINETIS_DMA_REQUEST_SRC_DAC0              45
#  define KINETIS_DMA_REQUEST_SRC_DAC1              46
#  define KINETIS_DMA_REQUEST_SRC_CMT               47
#  define KINETIS_DMA_REQUEST_SRC_PDB               48
#  define KINETIS_DMA_REQUEST_SRC_PCM_A             49
#  define KINETIS_DMA_REQUEST_SRC_PCM_B             50
#  define KINETIS_DMA_REQUEST_SRC_PCM_C             51
#  define KINETIS_DMA_REQUEST_SRC_PCM_D             52
#  define KINETIS_DMA_REQUEST_SRC_PCM_E             53
#  define KINETIS_DMA_REQUEST_SRC_TIMER0            54
#  define KINETIS_DMA_REQUEST_SRC_TIMER1            55
#  define KINETIS_DMA_REQUEST_SRC_TIMER2            56
#  define KINETIS_DMA_REQUEST_SRC_TIMER3            57

#endif /* KINETIS K64 */

#ifdef KINETIS_K66

#  define KINETIS_DMA_REQUEST_SRC_TSI0               1
#  define KINETIS_DMA_REQUEST_SRC_UART0_RX           2
#  define KINETIS_DMA_REQUEST_SRC_UART0_TX           3
#  define KINETIS_DMA_REQUEST_SRC_UART1_RX           4
#  define KINETIS_DMA_REQUEST_SRC_UART1_TX           5
#  define KINETIS_DMA_REQUEST_SRC_UART2_RX           6
#  define KINETIS_DMA_REQUEST_SRC_UART2_TX           7
#  define KINETIS_DMA_REQUEST_SRC_UART3_RX           8
#  define KINETIS_DMA_REQUEST_SRC_UART3_TX           9
#  define KINETIS_DMA_REQUEST_SRC_UART4_RXTX        10
#  define KINETIS_DMA_REQUEST_SRC_I2S0_RX           12
#  define KINETIS_DMA_REQUEST_SRC_I2S0_TX           13
#  define KINETIS_DMA_REQUEST_SRC_SPI0_RX           14
#  define KINETIS_DMA_REQUEST_SRC_SPI0_TX           15
#  define KINETIS_DMA_REQUEST_SRC_SPI1_RX           16
#  define KINETIS_DMA_REQUEST_SRC_SPI1_TX           17
#  define KINETIS_DMA_REQUEST_SRC_I2C0__I2C3        18
#  define KINETIS_DMA_REQUEST_SRC_I2C1__I2C2        19
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH0          20
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH1          21
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH2          22
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH3          23
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH4          24
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH5          25
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH6          26
#  define KINETIS_DMA_REQUEST_SRC_FTM0_CH7          27
#  define KINETIS_DMA_REQUEST_SRC_FTM1_TPM1_CH0     28
#  define KINETIS_DMA_REQUEST_SRC_FTM1_TPM1_CH1     29
#  define KINETIS_DMA_REQUEST_SRC_FTM2_TPM2_CH0     30
#  define KINETIS_DMA_REQUEST_SRC_FTM2_TPM2_CH1     31
#  define KINETIS_DMA_REQUEST_SRC_FTM3_CH0          32
#  define KINETIS_DMA_REQUEST_SRC_FTM3_CH1          33
#  define KINETIS_DMA_REQUEST_SRC_FTM3_CH2          34
#  define KINETIS_DMA_REQUEST_SRC_FTM3_CH3          35
#  define KINETIS_DMA_REQUEST_SRC_FTM3_CH4          36
#  define KINETIS_DMA_REQUEST_SRC_FTM3_CH5          37
#  define KINETIS_DMA_REQUEST_SRC_FTM3_CH6__SPI2_RX 38
#  define KINETIS_DMA_REQUEST_SRC_FTM3_CH7__SPI2_TX 39
#  define KINETIS_DMA_REQUEST_SRC_ADC0              40
#  define KINETIS_DMA_REQUEST_SRC_ADC1              41
#  define KINETIS_DMA_REQUEST_SRC_CMP0              42
#  define KINETIS_DMA_REQUEST_SRC_CMP1              43
#  define KINETIS_DMA_REQUEST_SRC_CMP2__CMP3        44
#  define KINETIS_DMA_REQUEST_SRC_DAC0              45
#  define KINETIS_DMA_REQUEST_SRC_DAC1              46
#  define KINETIS_DMA_REQUEST_SRC_CMT               47
#  define KINETIS_DMA_REQUEST_SRC_PDB               48
#  define KINETIS_DMA_REQUEST_SRC_PCM_A             49
#  define KINETIS_DMA_REQUEST_SRC_PCM_B             50
#  define KINETIS_DMA_REQUEST_SRC_PCM_C             51
#  define KINETIS_DMA_REQUEST_SRC_PCM_D             52
#  define KINETIS_DMA_REQUEST_SRC_PCM_E             53
#  define KINETIS_DMA_REQUEST_SRC_TIMER0            54
#  define KINETIS_DMA_REQUEST_SRC_TIMER1            55
#  define KINETIS_DMA_REQUEST_SRC_TIMER2            56
#  define KINETIS_DMA_REQUEST_SRC_TIMER3            57
#  define KINETIS_DMA_REQUEST_SRC_LPUART0_RX        58
#  define KINETIS_DMA_REQUEST_SRC_LPUART0_TX        59

#endif /* KINETIS_K66 */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_DMAMUX_H */

/****************************************************************************
 * arch/arm/include/cxd56xx/irq.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_IRQ_H
#define __ARCH_ARM_INCLUDE_CXD56XX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map
 * directly to bits in the NVIC. This does, however, waste several words of
 * memory in the IRQ to handle mapping tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define CXD56_IRQ_RESERVED       (0) /* Reserved vector (only used with CONFIG_DEBUG) */
                                     /* Vector  0: Reset stack pointer value */
                                     /* Vector  1: Reset (not handler as an IRQ) */
#define CXD56_IRQ_NMI            (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define CXD56_IRQ_HARDFAULT      (3) /* Vector  3: Hard fault */
#define CXD56_IRQ_MEMFAULT       (4) /* Vector  4: Memory management (MPU) */
#define CXD56_IRQ_BUSFAULT       (5) /* Vector  5: Bus fault */
#define CXD56_IRQ_USAGEFAULT     (6) /* Vector  6: Usage fault */
#define CXD56_IRQ_SIGNVALUE      (7) /* Vector  7: Sign value */
#define CXD56_IRQ_SVCALL        (11) /* Vector 11: SVC call */
#define CXD56_IRQ_DBGMONITOR    (12) /* Vector 12: Debug Monitor */
                                     /* Vector 13: Reserved */
#define CXD56_IRQ_PENDSV        (14) /* Vector 14: Pendable system service request */
#define CXD56_IRQ_SYSTICK       (15) /* Vector 15: System tick */
#define CXD56_IRQ_EXTINT        (16) /* Vector 16: Vector number of the first external interrupt */

/* Cortex-M4 External interrupts (vectors >= 16) */

#define CXD56_IRQ_PMU           (CXD56_IRQ_EXTINT+0)  /**< PMU IRQ number */
#define CXD56_IRQ_CRG           (CXD56_IRQ_EXTINT+1)  /**< CRG IRQ number */
#define CXD56_IRQ_HVDD          (CXD56_IRQ_EXTINT+2)  /**< HVDD IRQ number */
#define CXD56_IRQ_LP            (CXD56_IRQ_EXTINT+3)  /**< LP IRQ number */
#define CXD56_IRQ_RTC0_A0       (CXD56_IRQ_EXTINT+4)  /**< RTC0_A0 IRQ number */
#define CXD56_IRQ_RTC0_A1       (CXD56_IRQ_EXTINT+5)  /**< RTC0_A1 IRQ number */
#define CXD56_IRQ_RTC0_A2       (CXD56_IRQ_EXTINT+6)  /**< RTC0_A2 IRQ number */
#define CXD56_IRQ_RTC1_A0       (CXD56_IRQ_EXTINT+7)  /**< RTC1_A0 IRQ number */
#define CXD56_IRQ_RTC1_A1       (CXD56_IRQ_EXTINT+8)  /**< RTC1_A1 IRQ number */
#define CXD56_IRQ_RTC1_A2       (CXD56_IRQ_EXTINT+9)  /**< RTC1_A2 IRQ number */
#define CXD56_IRQ_RTC_INT       (CXD56_IRQ_EXTINT+10) /**< RTC_INT IRQ number */
#define CXD56_IRQ_UART1         (CXD56_IRQ_EXTINT+11) /**< UART1 IRQ number */
#define CXD56_IRQ_UART0         (CXD56_IRQ_EXTINT+12) /**< UART0 IRQ number */
#define CXD56_IRQ_HOSTIF_0      (CXD56_IRQ_EXTINT+13) /**< HOSTIF_0 IRQ number */
#define CXD56_IRQ_HOSTIF_1      (CXD56_IRQ_EXTINT+14) /**< HOSTIF_1 IRQ number */
#define CXD56_IRQ_HOSTIF_2      (CXD56_IRQ_EXTINT+15) /**< HOSTIF_2 IRQ number */
#define CXD56_IRQ_SCU_SPI       (CXD56_IRQ_EXTINT+16) /**< SCU_0 SPI IRQ number */
#define CXD56_IRQ_SCU_I2C0      (CXD56_IRQ_EXTINT+17) /**< SCU_1 I2C1 IRQ number */
#define CXD56_IRQ_SCU_I2C1      (CXD56_IRQ_EXTINT+18) /**< SCU_2 I2C2 IRQ number */
#define CXD56_IRQ_SCU_3         (CXD56_IRQ_EXTINT+19) /**< SCU_3 SCU IRQ number */
#define CXD56_IRQ_EXDEVICE_0    (CXD56_IRQ_EXTINT+20) /**< EXDEVICE_0 IRQ number */
#define CXD56_IRQ_EXDEVICE_1    (CXD56_IRQ_EXTINT+21) /**< EXDEVICE_1 IRQ number */
#define CXD56_IRQ_EXDEVICE_2    (CXD56_IRQ_EXTINT+22) /**< EXDEVICE_2 IRQ number */
#define CXD56_IRQ_EXDEVICE_3    (CXD56_IRQ_EXTINT+23) /**< EXDEVICE_3 IRQ number */
#define CXD56_IRQ_EXDEVICE_4    (CXD56_IRQ_EXTINT+24) /**< EXDEVICE_4 IRQ number */
#define CXD56_IRQ_EXDEVICE_5    (CXD56_IRQ_EXTINT+25) /**< EXDEVICE_5 IRQ number */
#define CXD56_IRQ_EXDEVICE_6    (CXD56_IRQ_EXTINT+26) /**< EXDEVICE_6 IRQ number */
#define CXD56_IRQ_EXDEVICE_7    (CXD56_IRQ_EXTINT+27) /**< EXDEVICE_7 IRQ number */
#define CXD56_IRQ_EXDEVICE_8    (CXD56_IRQ_EXTINT+28) /**< EXDEVICE_8 IRQ number */
#define CXD56_IRQ_EXDEVICE_9    (CXD56_IRQ_EXTINT+29) /**< EXDEVICE_9 IRQ number */
#define CXD56_IRQ_EXDEVICE_10   (CXD56_IRQ_EXTINT+30) /**< EXDEVICE_10 IRQ number */
#define CXD56_IRQ_EXDEVICE_11   (CXD56_IRQ_EXTINT+31) /**< EXDEVICE_11 IRQ number */
#define CXD56_IRQ_DMA_A_0       (CXD56_IRQ_EXTINT+32) /**< DMA_A_0 IRQ number */
#define CXD56_IRQ_DMA_A_1       (CXD56_IRQ_EXTINT+33) /**< DMA_A_1 IRQ number */
#define CXD56_IRQ_DMA_A_2       (CXD56_IRQ_EXTINT+34) /**< DMA_A_2 IRQ number */
#define CXD56_IRQ_DMA_A_3       (CXD56_IRQ_EXTINT+35) /**< DMA_A_3 IRQ number */
#define CXD56_IRQ_DMA_A_4       (CXD56_IRQ_EXTINT+36) /**< DMA_A_4 IRQ number */
#define CXD56_IRQ_DMA_A_5       (CXD56_IRQ_EXTINT+37) /**< DMA_A_5 IRQ number */
#define CXD56_IRQ_DMA_A_6       (CXD56_IRQ_EXTINT+38) /**< DMA_A_6 IRQ number */
#define CXD56_IRQ_DMA_A_7       (CXD56_IRQ_EXTINT+39) /**< DMA_A_7 IRQ number */
#define CXD56_IRQ_DMA_A_8       (CXD56_IRQ_EXTINT+40) /**< DMA_A_8 IRQ number */
#define CXD56_IRQ_DMA_A_9       (CXD56_IRQ_EXTINT+41) /**< DMA_A_9 IRQ number */
#define CXD56_IRQ_DMA_A_10      (CXD56_IRQ_EXTINT+42) /**< DMA_A_10 IRQ number */
#define CXD56_IRQ_DMA_A_11      (CXD56_IRQ_EXTINT+43) /**< DMA_A_11 IRQ number */
#define CXD56_IRQ_DMA_A_12      (CXD56_IRQ_EXTINT+44) /**< DMA_A_12 IRQ number */
#define CXD56_IRQ_DMA_A_13      (CXD56_IRQ_EXTINT+45) /**< DMA_A_13 IRQ number */
#define CXD56_IRQ_DMA_A_14      (CXD56_IRQ_EXTINT+46) /**< DMA_A_14 IRQ number */
#define CXD56_IRQ_DMA_A_15      (CXD56_IRQ_EXTINT+47) /**< DMA_A_15 IRQ number */
#define CXD56_IRQ_DMA_A_16      (CXD56_IRQ_EXTINT+48) /**< DMA_A_16 IRQ number */
#define CXD56_IRQ_DMA_A_17      (CXD56_IRQ_EXTINT+49) /**< DMA_A_17 IRQ number */
#define CXD56_IRQ_DMA_A_18      (CXD56_IRQ_EXTINT+50) /**< DMA_A_18 IRQ number */
#define CXD56_IRQ_DMA_A_19      (CXD56_IRQ_EXTINT+51) /**< DMA_A_19 IRQ number */
#define CXD56_IRQ_DMA_A_20      (CXD56_IRQ_EXTINT+52) /**< DMA_A_20 IRQ number */
#define CXD56_IRQ_DMA_A_21      (CXD56_IRQ_EXTINT+53) /**< DMA_A_21 IRQ number */
#define CXD56_IRQ_DMA_A_22      (CXD56_IRQ_EXTINT+54) /**< DMA_A_22 IRQ number */
#define CXD56_IRQ_DMA_A_23      (CXD56_IRQ_EXTINT+55) /**< DMA_A_23 IRQ number */
#define CXD56_IRQ_DMA_A_24      (CXD56_IRQ_EXTINT+56) /**< DMA_A_24 IRQ number */
#define CXD56_IRQ_DMA_A_25      (CXD56_IRQ_EXTINT+57) /**< DMA_A_25 IRQ number */
#define CXD56_IRQ_DMA_A_26      (CXD56_IRQ_EXTINT+58) /**< DMA_A_26 IRQ number */
#define CXD56_IRQ_DMA_A_27      (CXD56_IRQ_EXTINT+59) /**< DMA_A_27 IRQ number */
#define CXD56_IRQ_DMA_A_28      (CXD56_IRQ_EXTINT+60) /**< DMA_A_28 IRQ number */
#define CXD56_IRQ_DMA_A_29      (CXD56_IRQ_EXTINT+61) /**< DMA_A_29 IRQ number */
#define CXD56_IRQ_DMA_A_30      (CXD56_IRQ_EXTINT+62) /**< DMA_A_30 IRQ number */
#define CXD56_IRQ_DMA_A_31      (CXD56_IRQ_EXTINT+63) /**< DMA_A_31 IRQ number */
#define CXD56_IRQ_DMA_B_0       (CXD56_IRQ_EXTINT+64) /**< DMA_B_0 IRQ number */
#define CXD56_IRQ_DMA_B_1       (CXD56_IRQ_EXTINT+65) /**< DMA_B_1 IRQ number */
#define CXD56_IRQ_DMA_C_0       (CXD56_IRQ_EXTINT+66) /**< DMA_C_0 IRQ number */
#define CXD56_IRQ_DMA_C_1       (CXD56_IRQ_EXTINT+67) /**< DMA_C_1 IRQ number */
#define CXD56_IRQ_DMA_D_0       (CXD56_IRQ_EXTINT+68) /**< DMA_D_0 IRQ number */
#define CXD56_IRQ_DMA_D_1       (CXD56_IRQ_EXTINT+69) /**< DMA_D_1 IRQ number */
#define CXD56_IRQ_SAKE_NSEC     (CXD56_IRQ_EXTINT+70) /**< SAKE_NSEC IRQ number */
#define CXD56_IRQ_SAKE_SEC      (CXD56_IRQ_EXTINT+71) /**< SAKE_SEC IRQ number */
#define CXD56_IRQ_USB_VBUS      (CXD56_IRQ_EXTINT+72) /**< USB_VBUS IRQ number */
#define CXD56_IRQ_USB_VBUSN     (CXD56_IRQ_EXTINT+73) /**< USB_VBUSN IRQ number */
#define CXD56_IRQ_SPIM          (CXD56_IRQ_EXTINT+74) /**< SPI0 IRQ number */
#define CXD56_IRQ_I2CM          (CXD56_IRQ_EXTINT+75) /**< I2C0 IRQ number */
#define CXD56_IRQ_DEBUG0        (CXD56_IRQ_EXTINT+76) /**< DEBUG0 IRQ number */
#define CXD56_IRQ_DEBUG1        (CXD56_IRQ_EXTINT+77) /**< DEBUG1 IRQ number */
#define CXD56_IRQ_FIFO_TO       (CXD56_IRQ_EXTINT+78) /**< FIFO_TO IRQ number */
#define CXD56_IRQ_FIFO_FROM     (CXD56_IRQ_EXTINT+79) /**< FIFO_FROM IRQ number */
#define CXD56_IRQ_SPH0          (CXD56_IRQ_EXTINT+80) /**< SPH0 IRQ number */
#define CXD56_IRQ_SPH1          (CXD56_IRQ_EXTINT+81) /**< SPH1 IRQ number */
#define CXD56_IRQ_SPH2          (CXD56_IRQ_EXTINT+82) /**< SPH2 IRQ number */
#define CXD56_IRQ_SPH3          (CXD56_IRQ_EXTINT+83) /**< SPH3 IRQ number */
#define CXD56_IRQ_SPH4          (CXD56_IRQ_EXTINT+84) /**< SPH4 IRQ number */
#define CXD56_IRQ_SPH5          (CXD56_IRQ_EXTINT+85) /**< SPH5 IRQ number */
#define CXD56_IRQ_SPH6          (CXD56_IRQ_EXTINT+86) /**< SPH6 IRQ number */
#define CXD56_IRQ_SPH7          (CXD56_IRQ_EXTINT+87) /**< SPH7 IRQ number */
#define CXD56_IRQ_SPH8          (CXD56_IRQ_EXTINT+88) /**< SPH8 IRQ number */
#define CXD56_IRQ_SPH9          (CXD56_IRQ_EXTINT+89) /**< SPH9 IRQ number */
#define CXD56_IRQ_SPH10         (CXD56_IRQ_EXTINT+90) /**< SPH10 IRQ number */
#define CXD56_IRQ_SPH11         (CXD56_IRQ_EXTINT+91) /**< SPH11 IRQ number */
#define CXD56_IRQ_SPH12         (CXD56_IRQ_EXTINT+92) /**< SPH12 IRQ number */
#define CXD56_IRQ_SPH13         (CXD56_IRQ_EXTINT+93) /**< SPH13 IRQ number */
#define CXD56_IRQ_SPH14         (CXD56_IRQ_EXTINT+94) /**< SPH14 IRQ number */
#define CXD56_IRQ_SPH15         (CXD56_IRQ_EXTINT+95) /**< SPH15 IRQ number */
#define CXD56_IRQ_SW_INT        (CXD56_IRQ_EXTINT+96) /**< SW_INT IRQ number */
#define CXD56_IRQ_TIMER0        (CXD56_IRQ_EXTINT+97) /**< TIMER0 IRQ number */
#define CXD56_IRQ_TIMER1        (CXD56_IRQ_EXTINT+98) /**< TIMER1 IRQ number */
#define CXD56_IRQ_TIMER2        (CXD56_IRQ_EXTINT+99) /**< TIMER2 IRQ number */
#define CXD56_IRQ_WDT_INT       (CXD56_IRQ_EXTINT+100) /**< WDT_INT IRQ number */
#define CXD56_IRQ_WDT_RES       (CXD56_IRQ_EXTINT+101) /**< WDT_RES IRQ number */
#define CXD56_IRQ_AUDIO_0       (CXD56_IRQ_EXTINT+102) /**< AUDIO_0(MIC) IRQ number */
#define CXD56_IRQ_AUDIO_1       (CXD56_IRQ_EXTINT+103) /**< AUDIO_1(I2S1) IRQ number */
#define CXD56_IRQ_AUDIO_2       (CXD56_IRQ_EXTINT+104) /**< AUDIO_2(I2S2) IRQ number */
#define CXD56_IRQ_AUDIO_3       (CXD56_IRQ_EXTINT+105) /**< AUDIO_3(CODEC) IRQ number */
#define CXD56_IRQ_GE2D          (CXD56_IRQ_EXTINT+106) /**< APP_IMG 2D Graphics Engine IRQ number */
#define CXD56_IRQ_ROT           (CXD56_IRQ_EXTINT+107) /**< APP_IMG_ROTation IRQ number */
#define CXD56_IRQ_CISIF         (CXD56_IRQ_EXTINT+108) /**< APP_IMG CISIF IRQ number */
#define CXD56_IRQ_IMG_WSPI      (CXD56_IRQ_EXTINT+109) /**< APP_IMG WSSP IRQ number */
#define CXD56_IRQ_IDMAC         (CXD56_IRQ_EXTINT+110) /**< APP_IMG DMAC IRQ number */
#define CXD56_IRQ_APP_UART      (CXD56_IRQ_EXTINT+111) /**< APP_IMG UART IRQ number */
#define CXD56_IRQ_VSYNC         (CXD56_IRQ_EXTINT+112) /**< APP_IMG VSYNC IRQ number */
#define CXD56_IRQ_IMG_SPI       (CXD56_IRQ_EXTINT+113) /**< APP_IMG SSP IRQ number */
#define CXD56_IRQ_EMMC          (CXD56_IRQ_EXTINT+114) /**< APP_PER EMMC IRQ number */
#define CXD56_IRQ_SDIO          (CXD56_IRQ_EXTINT+115) /**< APP_PER SDIO IRQ number */
#define CXD56_IRQ_USB_INT       (CXD56_IRQ_EXTINT+116) /**< APP_PER USB_INT IRQ number */
#define CXD56_IRQ_USB_SYS       (CXD56_IRQ_EXTINT+117) /**< APP_PER USB_SYS IRQ number */
#define CXD56_IRQ_APP_DMAC0     (CXD56_IRQ_EXTINT+118) /**< APP_DMAC0 IRQ number */
#define CXD56_IRQ_APP_DMAC1     (CXD56_IRQ_EXTINT+119) /**< APP_DMAC1 IRQ number */
#define CXD56_IRQ_APP_SAKE_NSEC (CXD56_IRQ_EXTINT+120) /**< APP_SAKE_NSEC IRQ number */
#define CXD56_IRQ_APP_SAKE_SEC  (CXD56_IRQ_EXTINT+121) /**< APP_SAKE_SEC IRQ number */
#define CXD56_IRQ_SKDMAC_0      (CXD56_IRQ_EXTINT+122) /**< APP_SAKE_DMAC_0 IRQ number */
#define CXD56_IRQ_SKDMAC_1      (CXD56_IRQ_EXTINT+123) /**< APP_SAKE_DMAC_1 IRQ number */
#define CXD56_IRQ_APP_PPB       (CXD56_IRQ_EXTINT+124) /**< reserved */
#define CXD56_IRQ_GPS_OR        (CXD56_IRQ_EXTINT+125) /**< GNSS_OR IRQ number */
#define CXD56_IRQ_SFC           (CXD56_IRQ_EXTINT+126) /**< SFC IRQ number */
#define CXD56_IRQ_PMIC          (CXD56_IRQ_EXTINT+127) /**< PMIC IRQ number */

#define CXD56_IRQ_NEXTINT       (128)
#define CXD56_IRQ_NIRQS         (CXD56_IRQ_EXTINT+CXD56_IRQ_NEXTINT)

/* Total number of IRQ numbers (This will need to be revisited if/when the
 * Cortex-M0 is supported)
 */

#define NR_VECTORS              CXD56_IRQ_NIRQS
#define NR_IRQS                 CXD56_IRQ_NIRQS

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*vic_vector_t)(uint32_t *regs);
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_CXD56XX_IRQ_H */

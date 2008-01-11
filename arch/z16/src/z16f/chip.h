/************************************************************************************
 * arch/z16/src/z16f/chip.h
 * include/arch/chip/chip.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

#ifndef __Z16F_CHIP_H
#define __Z16F_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Hexadecimal Representation *******************************************************/

#ifdef __ASSEMBLY__
# define _HX32(w)   %##w
# define _HX8(b)    %##b
#else
# define _HX32(w)   0x##w
# define _HX8(b)    0x##b
#endif

/* Z16F Chip Variants ***************************************************************/
 
#if defined(CONFIG_ARCH_CHIP_Z16F2810)
# define Z16F_INVMEM_SIZE     (128*1024)
# define Z16F_IRAM_SIZE       (4*1024)
# undef  Z16F_HAVE_EXTMEM
# undef  Z16F_HAVE_GPIO_PORTJ
# undef  Z16F_HAVE_GPIO_PORTK
#elif defined(CONFIG_ARCH_CHIP_Z16F2811)
# define Z16F_INVMEM_SIZE     (128*1024)
# define Z16F_IRAM_SIZE       (4*1024)
# define Z16F_HAVE_EXTMEM     1
# define Z16F_HAVE_GPIO_PORTJ 1
# define Z16F_HAVE_GPIO_PORTK 1
#elif defined(CONFIG_ARCH_CHIP_Z16F3211)
# define Z16F_INVMEM_SIZE     (32*1024)
# define Z16F_IRAM_SIZE       (2*1024)
# define Z16F_HAVE_EXTMEM      1
#elif defined(CONFIG_ARCH_CHIP_Z16F6411)
# define Z16F_INVMEM_SIZE     (64*1024)
# define Z16F_IRAM_SIZE       (4*1024)
# define Z16F_HAVE_EXTMEM     1
#else
# error "Z16F chip variant not specified"
#endif

/* Memory areas**********************************************************************
 *
 * Internal non-volatile memory starts at address zero.  The size
 * of the internal non-volatile memory is chip-dependent.
 */
 
#define Z16F_INVMEM_BASE        _HX32(00000000)

/* Most chip variants support external memory */

#ifdef Z16F_HAVE_EXTMEM
#  define Z16F_EXTMEMCS0_BASE   _HX32(00020000) /* External memory at CS0 */
#  define Z16F_EXTMEMCS0_SIZE   _HX32(007e0000) /*   (actual depends on board) */
#  define Z16F_EXTMEMCS1_BASE   _HX32(ff800000) /* External memory at CS1 */
#  define Z16F_EXTMEMCS1_SIZE   _HX32(00700000) /*   (actual depends on board) */
#  define Z16F_EXTMEMCS2A_BASE  _HX32(fff00000) /* External memory at CS2 */
#  define Z16F_EXTMEMCS2A_SIZE  _HX32(000f8000) /*   (actual depends on board) */
#  define Z16F_EXTMEMCS2B_BASE  _HX32(ffffc000) /* External memory at CS2 */
#  define Z16F_EXTMEMCS2B_SIZE  _HX32(00000800) /*   (actual depends on board) */
#endif

/* Internal RAM always ends at 0xffbfff.  The IRAM base address depends
 * on the size of the IRAM supported by the chip.
 */
 
#define Z16F_IRAM_BASE          (_HX32(ffffc000) - Z16F_IRAM_SIZE)

/* External memory mapped peripherals, internal I/O memory and SFRS */

#define Z16F_EXTIO_BASE         _HX32(ffffc800) /* External peripherals CS3-5 */
#define Z16F_EXTIO_SIZE         _HX32(00001800)
#define Z16F_IIO_BASE           _HX32(ffffe000) /* Internal I/O memory and SFRs */
#define Z16F_IIO_SIZE           _HX32(00001fff)

/* GPIO Port A-K ********************************************************************/

#define Z16F_GPIOA_IN           _HX32(ffffe100) /*  8-bits: Port A Input Data */
#define Z16F_GPIOA_OUT          _HX32(ffffe101) /*  8-bits: Port A Output Data */
#define Z16F_GPIOA_DD           _HX32(ffffe102) /*  8-bits: Port A Data Direction */
#define Z16F_GPIOA_HDE          _HX32(ffffe103) /*  8-bits: Port A High Drive Enable */
#define Z16F_GPIOA_AF           _HX32(ffffe104) /* 16-bits: Port A Alternate Function */
#define Z16F_GPIOA_AFH          _HX32(ffffe104) /*  8-bits: Port A Alternate Function High */
#define Z16F_GPIOA_AFL          _HX32(ffffe105) /*  8-bits: Port A Alternate Function Low */
#define Z16F_GPIOA_OC           _HX32(ffffe106) /*  8-bits: Port A Output Control */
#define Z16F_GPIOA_PUE          _HX32(ffffe107) /*  8-bits: Port A Pull-Up Enable */
#define Z16F_GPIOA_SMRE         _HX32(ffffe108) /*  8-bits: Port A Stop Mode Recovery En */
#define Z16F_GPIOA_IMUX1        _HX32(ffffe10c) /*  8-bits: Port A IRQ Mux 1 */
#define Z16F_GPIOA_IMUX         _HX32(ffffe10e) /*  8-bits: Port A IRQ Mux */
#define Z16F_GPIOA_IEDGE        _HX32(ffffe10f) /*  8-bits: Port A IRQ Edge */

#define Z16F_GPIOB_IN           _HX32(ffffe110) /*  8-bits: Port B Input Data */
#define Z16F_GPIOB_OUT          _HX32(ffffe111) /*  8-bits: Port B Output Data */
#define Z16F_GPIOB_DD           _HX32(ffffe112) /*  8-bits: Port B Data Direction */
#define Z16F_GPIOB_HDE          _HX32(ffffe113) /*  8-bits: Port B High Drive Enable */
#define Z16F_GPIOB_AFL          _HX32(ffffe115) /*  8-bits: Port B Alternate Function Low */
#define Z16F_GPIOB_OC           _HX32(ffffe116) /*  8-bits: Port B Output Control */
#define Z16F_GPIOB_PUE          _HX32(ffffe117) /*  8-bits: Port B Pull-Up Enable */
#define Z16F_GPIOB_SMRE         _HX32(ffffe118) /*  8-bits: Port B Stop Mode Recovery En */

#define Z16F_GPIOC_IN           _HX32(ffffe120) /*  8-bits: Port C Input Data */
#define Z16F_GPIOC_OUT          _HX32(ffffe121) /*  8-bits: Port C Output Data */
#define Z16F_GPIOC_DD           _HX32(ffffe122) /*  8-bits: Port C Data Direction */
#define Z16F_GPIOC_HDE          _HX32(ffffe123) /*  8-bits: Port C High Drive Enable */
#define Z16F_GPIOC_AF           _HX32(ffffe124) /* 16-bits: Port C Alternate Function */
#define Z16F_GPIOC_AFH          _HX32(ffffe124) /*  8-bits: Port C Alternate Function High */
#define Z16F_GPIOC_AFL          _HX32(ffffe125) /*  8-bits: Port C Alternate Function Low */
#define Z16F_GPIOC_OC           _HX32(ffffe126) /*  8-bits: Port C Output Control */
#define Z16F_GPIOC_PUE          _HX32(ffffe127) /*  8-bits: Port C Pull-Up Enable */
#define Z16F_GPIOC_SMRE         _HX32(ffffe128) /*  8-bits: Port C Stop Mode Recovery En */
#define Z16F_GPIOC_IMUX         _HX32(ffffe12e) /*  8-bits: Port C IRQ Mux */

#define Z16F_GPIOD_IN           _HX32(ffffe130) /*  8-bits: Port D Input Data */
#define Z16F_GPIOD_OUT          _HX32(ffffe131) /*  8-bits: Port D Output Data */
#define Z16F_GPIOD_DD           _HX32(ffffe132) /*  8-bits: Port D Data Direction */
#define Z16F_GPIOD_HDE          _HX32(ffffe133) /*  8-bits: Port D High Drive Enable */
#define Z16F_GPIOD_AF           _HX32(ffffe134) /* 16-bits: Port D Alternate Function */
#define Z16F_GPIOD_AFH          _HX32(ffffe134) /*  8-bits: Port D Alternate Function High */
#define Z16F_GPIOD_AFL          _HX32(ffffe135) /*  8-bits: Port D Alternate Function Low */
#define Z16F_GPIOD_OC           _HX32(ffffe136) /*  8-bits: Port D Output Control */
#define Z16F_GPIOD_PUE          _HX32(ffffe137) /*  8-bits: Port D Pull-Up Enable */
#define Z16F_GPIOD_SMRE         _HX32(ffffe138) /*  8-bits: Port D Stop Mode Recovery En */

#define Z16F_GPIOE_IN           _HX32(ffffe140) /*  8-bits: Port E Input Data */
#define Z16F_GPIOE_OUT          _HX32(ffffe141) /*  8-bits: Port E Output Data */
#define Z16F_GPIOE_DD           _HX32(ffffe142) /*  8-bits: Port E Data Direction */
#define Z16F_GPIOE_HDE          _HX32(ffffe143) /*  8-bits: Port E High Drive Enable */
#define Z16F_GPIOE_OC           _HX32(ffffe146) /*  8-bits: Port E Output Control */
#define Z16F_GPIOE_PUE          _HX32(ffffe147) /*  8-bits: Port E Pull-Up Enable */
#define Z16F_GPIOE_SMRE         _HX32(ffffe148) /*  8-bits: Port E Stop Mode Recovery En */

#define Z16F_GPIOF_IN           _HX32(ffffe150) /*  8-bits: Port F Input Data */
#define Z16F_GPIOF_OUT          _HX32(ffffe151) /*  8-bits: Port F Output Data */
#define Z16F_GPIOF_DD           _HX32(ffffe152) /*  8-bits: Port F Data Direction */
#define Z16F_GPIOF_HDE          _HX32(ffffe153) /*  8-bits: Port F High Drive Enable */
#define Z16F_GPIOF_AFL          _HX32(ffffe155) /*  8-bits: Port F Alternate Function Low */
#define Z16F_GPIOF_OC           _HX32(ffffe156) /*  8-bits: Port F Output Control */
#define Z16F_GPIOF_PUE          _HX32(ffffe157) /*  8-bits: Port F Pull-Up Enable */
#define Z16F_GPIOF_SMRE         _HX32(ffffe158) /*  8-bits: Port F Stop Mode Recovery En */

#define Z16F_GPIOG_IN           _HX32(ffffe160) /*  8-bits: Port G Input Data */
#define Z16F_GPIOG_OUT          _HX32(ffffe161) /*  8-bits: Port G Output Data */
#define Z16F_GPIOG_DD           _HX32(ffffe162) /*  8-bits: Port G Data Direction */
#define Z16F_GPIOG_HDE          _HX32(ffffe163) /*  8-bits: Port G High Drive Enable */
#define Z16F_GPIOG_AFL          _HX32(ffffe165) /*  8-bits: Port G Alternate Function Low */
#define Z16F_GPIOG_OC           _HX32(ffffe166) /*  8-bits: Port G Output Control */
#define Z16F_GPIOG_PUE          _HX32(ffffe167) /*  8-bits: Port G Pull-Up Enable */
#define Z16F_GPIOG_SMRE         _HX32(ffffe168) /*  8-bits: Port G Stop Mode Recovery En */

#define Z16F_GPIOH_IN           _HX32(ffffe170) /*  8-bits: Port H Input Data */
#define Z16F_GPIOH_OUT          _HX32(ffffe171) /*  8-bits: Port H Output Data */
#define Z16F_GPIOH_DD           _HX32(ffffe172) /*  8-bits: Port H Data Direction */
#define Z16F_GPIOH_HDE          _HX32(ffffe173) /*  8-bits: Port H High Drive Enable */
#define Z16F_GPIOH_AF           _HX32(ffffe174) /* 16-bits: Port H Alternate Function */
#define Z16F_GPIOH_AFH          _HX32(ffffe174) /*  8-bits: Port H Alternate Function High */
#define Z16F_GPIOH_AFL          _HX32(ffffe175) /*  8-bits: Port H Alternate Function LOw */
#define Z16F_GPIOH_OC           _HX32(ffffe176) /*  8-bits: Port H Output Control */
#define Z16F_GPIOH_PUE          _HX32(ffffe177) /*  8-bits: Port H Pull-Up Enable */
#define Z16F_GPIOH_SMRE         _HX32(ffffe178) /*  8-bits: Port H Stop Mode Recovery En */

#ifdef Z16F_HAVE_GPIO_PORTJ
# define Z16F_GPIOJ_IN          _HX32(ffffe180) /*  8-bits: Port J Input Data */
# define Z16F_GPIOJ_OUT         _HX32(ffffe181) /*  8-bits: Port J Output Data */
# define Z16F_GPIOJ_DD          _HX32(ffffe182) /*  8-bits: Port J Data Direction */
# define Z16F_GPIOJ_HDE         _HX32(ffffe183) /*  8-bits: Port J High Drive Enable */
# define Z16F_GPIOJ_OC          _HX32(ffffe186) /*  8-bits: Port J Output Control */
# define Z16F_GPIOJ_PUE         _HX32(ffffe187) /*  8-bits: Port J Pull-Up Enable */
# define Z16F_GPIOJ_SMRE        _HX32(ffffe188) /*  8-bits: Port J Stop Mode Recovery En */
#endif

#ifdef Z16F_HAVE_GPIO_PORTK
# define Z16F_GPIOK_IN          _HX32(ffffe190) /*  8-bits: Port K Input Data */
# define Z16F_GPIOK_OUT         _HX32(ffffe191) /*  8-bits: Port K Output Data */
# define Z16F_GPIOK_DD          _HX32(ffffe192) /*  8-bits: Port K Data Direction */
# define Z16F_GPIOK_HDE         _HX32(ffffe193) /*  8-bits: Port K High Drive Enable */
# define Z16F_GPIOK_AFL         _HX32(ffffe195) /*  8-bits: Port K Alternate Function Low */
# define Z16F_GPIOK_OC          _HX32(ffffe196) /*  8-bits: Port K Output Control */
# define Z16F_GPIOK_PUE         _HX32(ffffe197) /*  8-bits: Port K Pull-Up Enable */
# define Z16F_GPIOK_SMRE        _HX32(ffffe198) /*  8-bits: Port K Stop Mode Recovery En */
#endif

/* UART0/1 registers ****************************************************************/

#define Z16F_UART0_TXD          _HX32(ffffe200) /*  8-bits: UART0 Transmit Data */
#define Z16F_UART0_RXD          _HX32(ffffe200) /*  8-bits: UART0 Receive Data */
#define Z16F_UART0_STAT0        _HX32(ffffe201) /*  8-bits: UART0 Status 0 */
#define Z16F_UART0_CTL          _HX32(ffffe202) /* 16-bits: UART0 Control */
#define Z16F_UART0_CTL0         _HX32(ffffe202) /*  8-bits: UART0 Control 0 */
#define Z16F_UART0_CTL1         _HX32(ffffe203) /*  8-bits: UART0 COntrol 1 */
#define Z16F_UART0_MDSTAT       _HX32(ffffe204) /*  8-bits: UART0 Mode Select & Status */
#define Z16F_UART0_ADDR         _HX32(ffffe205) /*  8-bits: UART0 Address Compare */
#define Z16F_UART0_BR           _HX32(ffffe206) /* 16-bits: UART0 Baud Rate */
#define Z16F_UART0_BRH          _HX32(ffffe206) /*  8-bits: UART0 Baud Rate High Byte */
#define Z16F_UART0_BRL          _HX32(ffffe207) /*  8-bits: UART0 Baud Rate Low Byte */

#define Z16F_UART1_TXD          _HX32(ffffe210) /*  8-bits: UART1 Transmit Data */
#define Z16F_UART1_RXD          _HX32(ffffe210) /*  8-bits: UART1 Receive Data */
#define Z16F_UART1_STAT0        _HX32(ffffe211) /*  8-bits: UART1 Status 0 */
#define Z16F_UART1_CTL          _HX32(ffffe212) /* 16-bits: UART1 Control */
#define Z16F_UART1_CTL0         _HX32(ffffe212) /*  8-bits: UART1 Control 0 */
#define Z16F_UART1_CTL1         _HX32(ffffe213) /*  8-bits: UART1 COntrol 1 */
#define Z16F_UART1_MDSTAT       _HX32(ffffe214) /*  8-bits: UART1 Mode Select & Status */
#define Z16F_UART1_ADDR         _HX32(ffffe215) /*  8-bits: UART1 Address Compare */
#define Z16F_UART1_BR           _HX32(ffffe216) /* 16-bits: UART1 Baud Rate */
#define Z16F_UART1_BRH          _HX32(ffffe216) /*  8-bits: UART1 Baud Rate High Byte */
#define Z16F_UART1_BRL          _HX32(ffffe217) /*  8-bits: UART1 Baud Rate Low Byte */

/* Control Registers  ***************************************************************/

#define Z16F_CNTRL_PCOV         _HX32(fffffe04) /* 32-bits: Program counter overflow */
#define Z16F_CNTRL_SPOV         _HX32(fffffe0c) /* 32-bits: Stack pointer overflow */
#define Z16F_CNTRL_FLAGS        _HX32(fffffe10) /*  8-bits: flags */
#define Z16F_CNTRL_CPUCTL       _HX32(fffffe12) /*  8-bits: CPU control */

/* Flag register bits ***************************************************************/

#define Z16F_CNTRL_FLAGS_C      _HX8(80)        /* Bit 7: Carry flag */
#define Z16F_CNTRL_FLAGS_Z      _HX8(40)        /* Bit 6: Zero flag */
#define Z16F_CNTRL_FLAGS_S      _HX8(20)        /* Bit 5: Sign flag */
#define Z16F_CNTRL_FLAGS_V      _HX8(10)        /* Bit 4: Overflow flag */
#define Z16F_CNTRL_FLAGS_B      _HX8(08)        /* Bit 3: Blank flag */
#define Z16F_CNTRL_FLAGS_F1     _HX8(04)        /* Bit 2: User flag 1 */
#define Z16F_CNTRL_FLAGS_CIRQE  _HX8(02)        /* Bit 1: Chained interrupt enable */
#define Z16F_CNTRL_FLAGS_IRQE   _HX8(01)        /* Bit 0: Master interrupt enable */

/* CPU control register bits ********************************************************/

                                                /* Bits 7-2: Reserved, must be zero */
                                                /* Bits 1-0: DMA bandwidth control */
#define Z16F_CNTRL_CPUCTL_BWALL _HX8(00)        /*   DMA can consume 100% bandwidth */
#define Z16F_CNTRL_CPUCTL_BW11  _HX8(01)        /*   DMA can do 1 transaction per 1 cycle */
#define Z16F_CNTRL_CPUCTL_BW12  _HX8(01)        /*   DMA can do 1 transaction per 2 cycles */
#define Z16F_CNTRL_CPUCTL_BW13  _HX8(01)        /*   DMA can do 1 transaction per 3 cycles */

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* The following two routines are called from the low-level reset logic.  z16f_lowinit()
 * must be provided by the board-specific logic; z16f_lowuartinit() is called only if
 * debugging support for up_lowputc (or getc) is enabled.
 */

extern void z16f_lowinit(void);
#if defined(CONFIG_ARCH_LOWPUTC) || defined(CONFIG_ARCH_LOWGETC)
extern void z16f_lowuartinit(void);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif  /* __Z16F_CHIP_H */

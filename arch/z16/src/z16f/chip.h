/************************************************************************************
 * arch/z16/src/z16f/chip.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ************************************************************************************/

#ifndef __ARCH_Z16_SRC_Z16F_CHIP_H
#define __ARCH_Z16_SRC_Z16F_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

#include <arch/irq.h>
#if !defined(__ASSEMBLY__) && defined(CONFIG_Z16F_ESPI)
#  include <nuttx/spi/spi.h>
#endif

#include "z16_internal.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Hexadecimal Representation *******************************************************/

#ifdef __ASSEMBLY__
# define _HX32(w)                 %##w
# define _HX8(b)                  %##b
#else
# define _HX32(w)                 0x##w
# define _HX8(b)                  0x##b
#endif

/* Z16F Chip Variants ***************************************************************/

#if defined(CONFIG_ARCH_CHIP_Z16F2810)
# define Z16F_INVMEM_SIZE         (128*1024)
# define Z16F_IRAM_SIZE           (4*1024)
# undef  Z16F_HAVE_EXTMEM
# undef  Z16F_HAVE_GPIO_PORTJ
# undef  Z16F_HAVE_GPIO_PORTK
#elif defined(CONFIG_ARCH_CHIP_Z16F2811)
# define Z16F_INVMEM_SIZE         (128*1024)
# define Z16F_IRAM_SIZE           (4*1024)
# define Z16F_HAVE_EXTMEM         1
# define Z16F_HAVE_GPIO_PORTJ     1
# define Z16F_HAVE_GPIO_PORTK     1
#elif defined(CONFIG_ARCH_CHIP_Z16F3211)
# define Z16F_INVMEM_SIZE         (32*1024)
# define Z16F_IRAM_SIZE           (2*1024)
# define Z16F_HAVE_EXTMEM         1
#elif defined(CONFIG_ARCH_CHIP_Z16F6411)
# define Z16F_INVMEM_SIZE         (64*1024)
# define Z16F_IRAM_SIZE           (4*1024)
# define Z16F_HAVE_EXTMEM         1
#else
# error "Z16F chip variant not specified"
#endif

/* Flash option settings at address 0x00000000 **************************************/

#define Z16F_FLOPTION0            rom char _flash_option0 _At 0x0
#define Z16F_FLOPTION1            rom char _flash_option1 _At 0x1
#define Z16F_FLOPTION2            rom char _flash_option2 _At 0x2
#define Z16F_FLOPTION3            rom char _flash_option3 _At 0x3

#define Z16F_FLOPTION0_OSCSEL     _HX8(c0)        /* Bits 6-7: OSC_SEL */
#define Z16F_FLOPTION0_EXTRNRC    _HX8(00)        /* 00: On-chip oscillator with ext RC networks */
#define Z16F_FLOPTION0_LOWFREQ    _HX8(40)        /* 01: Min. power with very low frequency crystals */
#define Z16F_FLOPTION0_MEDFREQ    _HX8(80)        /* 10: Medium power with medium frequency crystals */
#define Z16F_FLOPTION0_MAXPWR     _HX8(c0)        /* 11: Maximum power with high frequency crystals */
#define Z16F_FLOPTION0_WDTRES     _HX8(20)        /* Bit 5: WDT Reset */
#define Z16F_FLOPTION0_WDTA0      _HX8(10)        /* Bit 4: WDT Always On */
#define Z16F_FLOPTION0_VBOA0      _HX8(08)        /* Bit 3: Voltage Brown-Out Protection Always On */
#define Z16F_FLOPTION0_DBGUART    _HX8(04)        /* Bit 2: Debug UART Enable */
#define Z16F_FLOPTION0_FWP        _HX8(02)        /* Bit 1: Flash Write Protect */
#define Z16F_FLOPTION0_RP         _HX8(01)        /* Bit 0: Read Protect */

#define Z16F_FLOPTION1_RESVD      _HX8(f8)        /* Bits 3-7: Reserved */
#define Z16F_FLOPTION1_MCEN       _HX8(04)        /* Bit 2: Motor control pins enable */
#define Z16F_FLOPTION1_OFFH       _HX8(02)        /* Bit 1: High side OFF */
#define Z16F_FLOPTION1_OFFL       _HX8(01)        /* Bit 0: Low side OFF */

#define Z16F_FLOPTION2_RESVD      _HX8(ff)        /* Bits 0-7: reserved */

#define Z16F_FLOPTION3_ROMLESS    _HX8(80)        /* Bit 7: ROMLESS 16 Select */
#define Z16F_FLOPTION3_NORMAL     _HX8(40)        /* Bit 6: 1:Normal 0:Low power mode */
#define Z16F_FLOPTION3_RESVD      _HX8(3f)        /* Bits 0-5: Reserved */

/* Memory areas *********************************************************************
 *
 * Internal non-volatile memory starts at address zero.  The size
 * of the internal non-volatile memory is chip-dependent.
 */

#define Z16F_INVMEM_BASE          _HX32(00000000)

/* Most chip variants support external memory */

#ifdef Z16F_HAVE_EXTMEM
#  define Z16F_EXTMEMCS0_BASE     _HX32(00000000) /* External memory at CS0 */
#  define Z16F_EXTMEMCS0_SIZE     _HX32(007e0000) /*   (actual depends on board) */
#  define Z16F_EXTMEMCS1_BASE     _HX32(ff800000) /* External memory at CS1 */
#  define Z16F_EXTMEMCS1_SIZE     _HX32(00700000) /*   (actual depends on board) */
#  define Z16F_EXTMEMCS2A_BASE    _HX32(fff00000) /* External memory at CS2 */
#  define Z16F_EXTMEMCS2A_SIZE    _HX32(000f8000) /*   (actual depends on board) */
#  define Z16F_EXTMEMCS2B_BASE    _HX32(ffffc000) /* External memory at CS2 */
#  define Z16F_EXTMEMCS2B_SIZE    _HX32(00000800) /*   (actual depends on board) */
#endif

/* Internal RAM always ends at 0xffbfff.  The IRAM base address depends
 * on the size of the IRAM supported by the chip.
 */

#define Z16F_IRAM_BASE            (_HX32(ffffc000) - Z16F_IRAM_SIZE)

/* External memory mapped peripherals, internal I/O memory and SFRS */

#define Z16F_EXTIO_BASE           _HX32(ffffc800) /* External peripherals CS3-5 */
#define Z16F_EXTIO_SIZE           _HX32(00001800)
#define Z16F_IIO_BASE             _HX32(ffffe000) /* Internal I/O memory and SFRs */
#define Z16F_IIO_SIZE             _HX32(00001fff)

/* Control Registers  ***************************************************************/

#define Z16F_CNTRL_PCOV           _HX32(ffffe004) /* 32-bits: Program counter overflow */
#define Z16F_CNTRL_SPOV           _HX32(ffffe00c) /* 32-bits: Stack pointer overflow */
#define Z16F_CNTRL_FLAGS          _HX32(ffffe100) /*  8-bits: flags */
#define Z16F_CNTRL_CPUCTL         _HX32(ffffe102) /*  8-bits: CPU control */

/* Flag register bits ***************************************************************/

#define Z16F_CNTRL_FLAGS_C        _HX8(80)        /* Bit 7: Carry flag */
#define Z16F_CNTRL_FLAGS_Z        _HX8(40)        /* Bit 6: Zero flag */
#define Z16F_CNTRL_FLAGS_S        _HX8(20)        /* Bit 5: Sign flag */
#define Z16F_CNTRL_FLAGS_V        _HX8(10)        /* Bit 4: Overflow flag */
#define Z16F_CNTRL_FLAGS_B        _HX8(08)        /* Bit 3: Blank flag */
#define Z16F_CNTRL_FLAGS_F1       _HX8(04)        /* Bit 2: User flag 1 */
#define Z16F_CNTRL_FLAGS_CIRQE    _HX8(02)        /* Bit 1: Chained interrupt enable */
#define Z16F_CNTRL_FLAGS_IRQE     _HX8(01)        /* Bit 0: Master interrupt enable */

/* CPU control register bits ********************************************************/

                                                  /* Bits 7-2: Reserved, must be
                                                   * zero
                                                   */

                                                  /* Bits 1-0: DMA bandwidth
                                                   *   control
                                                   */

#define Z16F_CNTRL_CPUCTL_BWALL   _HX8(00)        /*   DMA can consume 100% bandwidth */
#define Z16F_CNTRL_CPUCTL_BW11    _HX8(01)        /*   DMA can do 1 transaction per 1 cycle */
#define Z16F_CNTRL_CPUCTL_BW12    _HX8(01)        /*   DMA can do 1 transaction per 2 cycles */
#define Z16F_CNTRL_CPUCTL_BW13    _HX8(01)        /*   DMA can do 1 transaction per 3 cycles */

/* Trace registers ******************************************************************/

#define Z16F_TRACE_CTL            _HX32(ffffe013) /*  8-bit: Trace Control */
#define Z16F_TRACE_ADDR           _HX32(ffffe014) /* 32-bit: Trace Address */

/* Interrupt controller registers ***************************************************/

#define Z16F_SYSEXCP              _HX32(ffffe020) /* 16-bit: System Exception Status */
#  define Z16F_SYSEXCPH           _HX32(ffffe020) /*  8-bit: System Exception Status High */
#  define Z16F_SYSEXCPL           _HX32(ffffe021) /*  8-bit: System Exception Status Low */
#define Z16F_LASTIRQ              _HX32(ffffe023) /*  8-bit: Last IRQ Register */
#define Z16F_IRQ0                 _HX32(ffffe030) /*  8-bit: Interrupt Request 0 */
#define Z16F_IRQ0_SET             _HX32(ffffe031) /*  8-bit: Interrupt Request 0 Set */
#define Z16F_IRQ0_EN              _HX32(ffffe032) /* 16-bit: IRQ0 Enable */
#  define Z16F_IRQ0_ENH           _HX32(ffffe032) /*  8-bit: IRQ0 Enable High Bit */
#  define Z16F_IRQ0_ENL           _HX32(ffffe033) /*  8-bit: IRQ0 Enable Low Bit */
#define Z16F_IRQ1                 _HX32(ffffe034) /*  8-bit: Interrupt Request 1 */
#define Z16F_IRQ1_SET             _HX32(ffffe035) /*  8-bit: Interrupt Request 1 Set */
#define Z16F_IRQ1_EN              _HX32(ffffe036) /* 16-bit: IRQ1 Enable */
#  define Z16F_IRQ1_ENH           _HX32(ffffe036) /*  8-bit: IRQ1 Enable High Bit */
#  define Z16F_IRQ1_ENL           _HX32(ffffe037) /*  8-bit: IRQ1 Enable Low Bit */
#define Z16F_IRQ2                 _HX32(ffffe038) /*  8-bit: Interrupt Request 2 */
#define Z16F_IRQ2_SET             _HX32(ffffe039) /*  8-bit  Interrupt Request 2 Set */
#define Z16F_IRQ2_EN              _HX32(ffffe03a) /* 16-bit: IRQ2 Enable */
#  define Z16F_IRQ2_ENH           _HX32(ffffe03a) /*  8-bit: IRQ2 Enable High Bit */
#  define Z16F_IRQ2_ENL           _HX32(ffffe03c) /*  8-bit: IRQ2 Enable Low Bit */

/* System exception status register bit definitions *********************************/

#define Z16F_SYSEXCPH_SPOVF       _HX8(80)        /* Bit 7: Stack pointer overflow */
#define Z16F_SYSEXCPH_PCOVF       _HX8(40)        /* Bit 6: Program counter overflow */
#define Z16F_SYSEXCPH_DIV0        _HX8(20)        /* Bit 5: Divide by zero */
#define Z16F_SYSEXCPH_DIVOVF      _HX8(10)        /* Bit 4: Divide overflow */
#define Z16F_SYSEXCPH_ILL         _HX8(08)        /* Bit 3: Illegal instruction */
                                                  /* Bits 0-2: Reserved */
                                                  /* Bits 3-7: Reserved */
#define Z16F_SYSEXCPL_WDTOSC      _HX8(04)        /* Bit 2: WDT oscillator failure */
#define Z16F_SYSEXCPL_PRIOSC      _HX8(02)        /* Bit 1: Primary oscillator failure */
#define Z16F_SYSEXCPL_WDT         _HX8(01)        /* Bit 0: Watchdog timer interrupt */

#define Z16F_SYSEXCP_SPOVF        (Z16F_SYSEXCPH_SPOVF << 8)
#define Z16F_SYSEXCP_PCOVF        (Z16F_SYSEXCPH_PCOVF << 8)
#define Z16F_SYSEXCP_DIV0         (Z16F_SYSEXCPH_DIV0 << 8)
#define Z16F_SYSEXCP_DIVOVF       (Z16F_SYSEXCPH_DIVOVF << 8)
#define Z16F_SYSEXCP_ILL          (Z16F_SYSEXCPH_ILL << 8)
#define Z16F_SYSEXCP_WDTOSC       Z16F_SYSEXCPL_WDTOSC
#define Z16F_SYSEXCP_PRIOSC       Z16F_SYSEXCPL_PRIOSC
#define Z16F_SYSEXCP_WDT          Z16F_SYSEXCPL_WDT

/* External memory interface ********************************************************/

#define Z16F_EXTCT                _HX32(ffffe070) /* External Interface Control */
#define Z16F_EXTCS0               _HX32(ffffe072) /* Chip Select 0 Control */
#  define Z16F_EXTCS0H            _HX32(ffffe072) /* Chip Select 0 Control High */
#  define Z16F_EXTCS0L            _HX32(ffffe073) /* Chip Select 0 Control Low */
#define Z16F_EXTCS1               _HX32(ffffe074) /* Chip Select 1 Control */
#  define Z16F_EXTCS1H            _HX32(ffffe074) /* Chip Select 1 Control High */
#  define Z16F_EXTCS1L            _HX32(ffffe075) /* Chip Select 1 Control Low */
#define Z16F_EXTCS2               _HX32(ffffe076) /* Chip Select 2 Control */
#  define Z16F_EXTCS2H            _HX32(ffffe076) /* Chip Select 2 Control High */
#  define Z16F_EXTCS2L            _HX32(ffffe077) /* Chip Select 2 Control Low */
#define Z16F_EXTCS3               _HX32(ffffe078) /* Chip Select 3 Control */
#  define Z16F_EXTCS3H            _HX32(ffffe078) /* Chip Select 3 Control High */
#  define Z16F_EXTCS3L            _HX32(ffffe079) /* Chip Select 3 Control Low */
#define Z16F_EXTCS4               _HX32(ffffe07a) /* Chip Select 4 Control  */
#  define Z16F_EXTCS4H            _HX32(ffffe07a) /* Chip Select 4 Control High  */
#  define Z16F_EXTCS4L            _HX32(ffffe07b) /* Chip Select 4 Control Low */
#define Z16F_EXTCS5               _HX32(ffffe07c) /* Chip Select 5 Control */
#  define Z16F_EXTCS5H            _HX32(ffffe07c) /* Chip Select 5 Control High */
#  define Z16F_EXTCS5L            _HX32(ffffe07d) /* Chip Select 5 Control Low */

/* Oscillator control registers *****************************************************/

#define Z16F_OSC_CTL              _HX32(ffffe0A0) /*  8-bit: Oscillator Control */
#define Z16F_OSC_DIV              _HX32(ffffe0A1) /*  8-bit: Oscillator Divide */

/* Oscillator control register bits *************************************************/

#define Z16F_OSCCTL_INTEN         _HX8(80)        /* Bit 7: Internal oscillator enabled */
#define Z16F_OSCCTL_XTLEN         _HX8(40)        /* Bit 6: Crystal oscillator enabled */
#define Z16F_OSCCTL_WDTEN         _HX8(20)        /* Bit 5: Watchdog timer enabled */
#define Z16F_OSCCTL_POFEN         _HX8(10)        /* Bit 4: Failure detection enabled */
#define Z16F_OSCCTL_WDFEN         _HX8(08)        /* Bit 3: WD Failure detection enabled*/
#define Z16F_OSCCTL_FLPEN         _HX8(04)        /* Bit 2: Flash low power enabled */
#define Z16F_OSCCTL_INT56         _HX8(00)        /* Bits 0-1=0: Internal 5.6 MHz */
#define Z16F_OSCCTL_EXTCLK        _HX8(02)        /* Bits 0-1=2: External clock */
#define Z16F_OSCCTL_WDT10KHZ      _HX8(03)        /* Bits 0-1=3: WD Timer 10 KHz*/

/* GPIO Port A-K ********************************************************************/

#define Z16F_GPIOA_IN             _HX32(ffffe100) /*  8-bits: Port A Input Data */
#define Z16F_GPIOA_OUT            _HX32(ffffe101) /*  8-bits: Port A Output Data */
#define Z16F_GPIOA_DD             _HX32(ffffe102) /*  8-bits: Port A Data Direction */
#define Z16F_GPIOA_HDE            _HX32(ffffe103) /*  8-bits: Port A High Drive Enable */
#define Z16F_GPIOA_AF             _HX32(ffffe104) /* 16-bits: Port A Alternate Function */
#  define Z16F_GPIOA_AFH          _HX32(ffffe104) /*  8-bits: Port A Alternate Function High */
#  define Z16F_GPIOA_AFL          _HX32(ffffe105) /*  8-bits: Port A Alternate Function Low */
#define Z16F_GPIOA_OC             _HX32(ffffe106) /*  8-bits: Port A Output Control */
#define Z16F_GPIOA_PUE            _HX32(ffffe107) /*  8-bits: Port A Pull-Up Enable */
#define Z16F_GPIOA_SMRE           _HX32(ffffe108) /*  8-bits: Port A Stop Mode Recovery En */
#define Z16F_GPIOA_IMUX1          _HX32(ffffe10c) /*  8-bits: Port A IRQ Mux 1 */
#define Z16F_GPIOA_IMUX           _HX32(ffffe10e) /*  8-bits: Port A IRQ Mux */
#define Z16F_GPIOA_IEDGE          _HX32(ffffe10f) /*  8-bits: Port A IRQ Edge */

#define Z16F_GPIOB_IN             _HX32(ffffe110) /*  8-bits: Port B Input Data */
#define Z16F_GPIOB_OUT            _HX32(ffffe111) /*  8-bits: Port B Output Data */
#define Z16F_GPIOB_DD             _HX32(ffffe112) /*  8-bits: Port B Data Direction */
#define Z16F_GPIOB_HDE            _HX32(ffffe113) /*  8-bits: Port B High Drive Enable */
#define Z16F_GPIOB_AFL            _HX32(ffffe115) /*  8-bits: Port B Alternate Function Low */
#define Z16F_GPIOB_OC             _HX32(ffffe116) /*  8-bits: Port B Output Control */
#define Z16F_GPIOB_PUE            _HX32(ffffe117) /*  8-bits: Port B Pull-Up Enable */
#define Z16F_GPIOB_SMRE           _HX32(ffffe118) /*  8-bits: Port B Stop Mode Recovery En */

#define Z16F_GPIOC_IN             _HX32(ffffe120) /*  8-bits: Port C Input Data */
#define Z16F_GPIOC_OUT            _HX32(ffffe121) /*  8-bits: Port C Output Data */
#define Z16F_GPIOC_DD             _HX32(ffffe122) /*  8-bits: Port C Data Direction */
#define Z16F_GPIOC_HDE            _HX32(ffffe123) /*  8-bits: Port C High Drive Enable */
#define Z16F_GPIOC_AF             _HX32(ffffe124) /* 16-bits: Port C Alternate Function */
#  define Z16F_GPIOC_AFH          _HX32(ffffe124) /*  8-bits: Port C Alternate Function High */
#  define Z16F_GPIOC_AFL          _HX32(ffffe125) /*  8-bits: Port C Alternate Function Low */
#define Z16F_GPIOC_OC             _HX32(ffffe126) /*  8-bits: Port C Output Control */
#define Z16F_GPIOC_PUE            _HX32(ffffe127) /*  8-bits: Port C Pull-Up Enable */
#define Z16F_GPIOC_SMRE           _HX32(ffffe128) /*  8-bits: Port C Stop Mode Recovery En */
#define Z16F_GPIOC_IMUX           _HX32(ffffe12e) /*  8-bits: Port C IRQ Mux */

#define Z16F_GPIOD_IN             _HX32(ffffe130) /*  8-bits: Port D Input Data */
#define Z16F_GPIOD_OUT            _HX32(ffffe131) /*  8-bits: Port D Output Data */
#define Z16F_GPIOD_DD             _HX32(ffffe132) /*  8-bits: Port D Data Direction */
#define Z16F_GPIOD_HDE            _HX32(ffffe133) /*  8-bits: Port D High Drive Enable */
#define Z16F_GPIOD_AF             _HX32(ffffe134) /* 16-bits: Port D Alternate Function */
#  define Z16F_GPIOD_AFH          _HX32(ffffe134) /*  8-bits: Port D Alternate Function High */
#  define Z16F_GPIOD_AFL          _HX32(ffffe135) /*  8-bits: Port D Alternate Function Low */
#define Z16F_GPIOD_OC             _HX32(ffffe136) /*  8-bits: Port D Output Control */
#define Z16F_GPIOD_PUE            _HX32(ffffe137) /*  8-bits: Port D Pull-Up Enable */
#define Z16F_GPIOD_SMRE           _HX32(ffffe138) /*  8-bits: Port D Stop Mode Recovery En */

#define Z16F_GPIOE_IN             _HX32(ffffe140) /*  8-bits: Port E Input Data */
#define Z16F_GPIOE_OUT            _HX32(ffffe141) /*  8-bits: Port E Output Data */
#define Z16F_GPIOE_DD             _HX32(ffffe142) /*  8-bits: Port E Data Direction */
#define Z16F_GPIOE_HDE            _HX32(ffffe143) /*  8-bits: Port E High Drive Enable */
#define Z16F_GPIOE_OC             _HX32(ffffe146) /*  8-bits: Port E Output Control */
#define Z16F_GPIOE_PUE            _HX32(ffffe147) /*  8-bits: Port E Pull-Up Enable */
#define Z16F_GPIOE_SMRE           _HX32(ffffe148) /*  8-bits: Port E Stop Mode Recovery En */

#define Z16F_GPIOF_IN             _HX32(ffffe150) /*  8-bits: Port F Input Data */
#define Z16F_GPIOF_OUT            _HX32(ffffe151) /*  8-bits: Port F Output Data */
#define Z16F_GPIOF_DD             _HX32(ffffe152) /*  8-bits: Port F Data Direction */
#define Z16F_GPIOF_HDE            _HX32(ffffe153) /*  8-bits: Port F High Drive Enable */
#define Z16F_GPIOF_AFL            _HX32(ffffe155) /*  8-bits: Port F Alternate Function Low */
#define Z16F_GPIOF_OC             _HX32(ffffe156) /*  8-bits: Port F Output Control */
#define Z16F_GPIOF_PUE            _HX32(ffffe157) /*  8-bits: Port F Pull-Up Enable */
#define Z16F_GPIOF_SMRE           _HX32(ffffe158) /*  8-bits: Port F Stop Mode Recovery En */

#define Z16F_GPIOG_IN             _HX32(ffffe160) /*  8-bits: Port G Input Data */
#define Z16F_GPIOG_OUT            _HX32(ffffe161) /*  8-bits: Port G Output Data */
#define Z16F_GPIOG_DD             _HX32(ffffe162) /*  8-bits: Port G Data Direction */
#define Z16F_GPIOG_HDE            _HX32(ffffe163) /*  8-bits: Port G High Drive Enable */
#define Z16F_GPIOG_AFL            _HX32(ffffe165) /*  8-bits: Port G Alternate Function Low */
#define Z16F_GPIOG_OC             _HX32(ffffe166) /*  8-bits: Port G Output Control */
#define Z16F_GPIOG_PUE            _HX32(ffffe167) /*  8-bits: Port G Pull-Up Enable */
#define Z16F_GPIOG_SMRE           _HX32(ffffe168) /*  8-bits: Port G Stop Mode Recovery En */

#define Z16F_GPIOH_IN             _HX32(ffffe170) /*  8-bits: Port H Input Data */
#define Z16F_GPIOH_OUT            _HX32(ffffe171) /*  8-bits: Port H Output Data */
#define Z16F_GPIOH_DD             _HX32(ffffe172) /*  8-bits: Port H Data Direction */
#define Z16F_GPIOH_HDE            _HX32(ffffe173) /*  8-bits: Port H High Drive Enable */
#define Z16F_GPIOH_AF             _HX32(ffffe174) /* 16-bits: Port H Alternate Function */
#  define Z16F_GPIOH_AFH          _HX32(ffffe174) /*  8-bits: Port H Alternate Function High */
#  define Z16F_GPIOH_AFL          _HX32(ffffe175) /*  8-bits: Port H Alternate Function LOw */
#define Z16F_GPIOH_OC             _HX32(ffffe176) /*  8-bits: Port H Output Control */
#define Z16F_GPIOH_PUE            _HX32(ffffe177) /*  8-bits: Port H Pull-Up Enable */
#define Z16F_GPIOH_SMRE           _HX32(ffffe178) /*  8-bits: Port H Stop Mode Recovery En */

#ifdef Z16F_HAVE_GPIO_PORTJ
# define Z16F_GPIOJ_IN            _HX32(ffffe180) /*  8-bits: Port J Input Data */
# define Z16F_GPIOJ_OUT           _HX32(ffffe181) /*  8-bits: Port J Output Data */
# define Z16F_GPIOJ_DD            _HX32(ffffe182) /*  8-bits: Port J Data Direction */
# define Z16F_GPIOJ_HDE           _HX32(ffffe183) /*  8-bits: Port J High Drive Enable */
# define Z16F_GPIOJ_OC            _HX32(ffffe186) /*  8-bits: Port J Output Control */
# define Z16F_GPIOJ_PUE           _HX32(ffffe187) /*  8-bits: Port J Pull-Up Enable */
# define Z16F_GPIOJ_SMRE          _HX32(ffffe188) /*  8-bits: Port J Stop Mode Recovery En */
#endif

#ifdef Z16F_HAVE_GPIO_PORTK
# define Z16F_GPIOK_IN            _HX32(ffffe190) /*  8-bits: Port K Input Data */
# define Z16F_GPIOK_OUT           _HX32(ffffe191) /*  8-bits: Port K Output Data */
# define Z16F_GPIOK_DD            _HX32(ffffe192) /*  8-bits: Port K Data Direction */
# define Z16F_GPIOK_HDE           _HX32(ffffe193) /*  8-bits: Port K High Drive Enable */
# define Z16F_GPIOK_AFL           _HX32(ffffe195) /*  8-bits: Port K Alternate Function Low */
# define Z16F_GPIOK_OC            _HX32(ffffe196) /*  8-bits: Port K Output Control */
# define Z16F_GPIOK_PUE           _HX32(ffffe197) /*  8-bits: Port K Pull-Up Enable */
# define Z16F_GPIOK_SMRE          _HX32(ffffe198) /*  8-bits: Port K Stop Mode Recovery En */
#endif

/* UART Register Offsets ************************************************************/

#define Z16F_UART_TXD             _HX8(00)        /*  8-bits: UART Transmit Data */
#define Z16F_UART_RXD             _HX8(00)        /*  8-bits: UART Receive Data */
#define Z16F_UART_STAT0           _HX8(01)        /*  8-bits: UART Status 0 */
#define Z16F_UART_CTL             _HX8(02)        /* 16-bits: UART Control */
#  define Z16F_UART_CTL0          _HX8(02)        /*  8-bits: UART Control 0 */
#  define Z16F_UART_CTL1          _HX8(03)        /*  8-bits: UART COntrol 1 */
#define Z16F_UART_MDSTAT          _HX8(04)        /*  8-bits: UART Mode Select & Status */
#define Z16F_UART_ADDR            _HX8(05)        /*  8-bits: UART Address Compare */
#define Z16F_UART_BR              _HX8(06)        /* 16-bits: UART Baud Rate */
#  define Z16F_UART_BRH           _HX8(06)        /*  8-bits: UART Baud Rate High Byte */
#  define Z16F_UART_BRL           _HX8(07)        /*  8-bits: UART Baud Rate Low Byte */

#define Z16F_UART0_BASE           _HX32(ffffe200) /* UART0 Register Base Address */
#define Z16F_UART1_BASE           _HX32(ffffe210) /* UART1 Register Base Address */

/* UART0/1 Registers ****************************************************************/

#define Z16F_UART0_TXD            _HX32(ffffe200) /*  8-bits: UART0 Transmit Data */
#define Z16F_UART0_RXD            _HX32(ffffe200) /*  8-bits: UART0 Receive Data */
#define Z16F_UART0_STAT0          _HX32(ffffe201) /*  8-bits: UART0 Status 0 */
#define Z16F_UART0_CTL            _HX32(ffffe202) /* 16-bits: UART0 Control */
#  define Z16F_UART0_CTL0         _HX32(ffffe202) /*  8-bits: UART0 Control 0 */
#  define Z16F_UART0_CTL1         _HX32(ffffe203) /*  8-bits: UART0 COntrol 1 */
#define Z16F_UART0_MDSTAT         _HX32(ffffe204) /*  8-bits: UART0 Mode Select & Status */
#define Z16F_UART0_ADDR           _HX32(ffffe205) /*  8-bits: UART0 Address Compare */
#define Z16F_UART0_BR             _HX32(ffffe206) /* 16-bits: UART0 Baud Rate */
#  define Z16F_UART0_BRH          _HX32(ffffe206) /*  8-bits: UART0 Baud Rate High Byte */
#  define Z16F_UART0_BRL          _HX32(ffffe207) /*  8-bits: UART0 Baud Rate Low Byte */

#define Z16F_UART1_TXD            _HX32(ffffe210) /*  8-bits: UART1 Transmit Data */
#define Z16F_UART1_RXD            _HX32(ffffe210) /*  8-bits: UART1 Receive Data */
#define Z16F_UART1_STAT0          _HX32(ffffe211) /*  8-bits: UART1 Status 0 */
#define Z16F_UART1_CTL            _HX32(ffffe212) /* 16-bits: UART1 Control */
#  define Z16F_UART1_CTL0         _HX32(ffffe212) /*  8-bits: UART1 Control 0 */
#  define Z16F_UART1_CTL1         _HX32(ffffe213) /*  8-bits: UART1 COntrol 1 */
#define Z16F_UART1_MDSTAT         _HX32(ffffe214) /*  8-bits: UART1 Mode Select & Status */
#define Z16F_UART1_ADDR           _HX32(ffffe215) /*  8-bits: UART1 Address Compare */
#define Z16F_UART1_BR             _HX32(ffffe216) /* 16-bits: UART1 Baud Rate */
#  define Z16F_UART1_BRH          _HX32(ffffe216) /*  8-bits: UART1 Baud Rate High Byte */
#  define Z16F_UART1_BRL          _HX32(ffffe217) /*  8-bits: UART1 Baud Rate Low Byte */

/* UART0/1 Status 0 Register Bit Definitions ****************************************/

#define Z16F_UARTSTAT0_RDA        _HX8(80)        /* Bit 7: Receive Data Available */
#define Z16F_UARTSTAT0_PE         _HX8(40)        /* Bit 6: Parity Error */
#define Z16F_UARTSTAT0_OE         _HX8(20)        /* Bit 5: Overrun Error */
#define Z16F_UARTSTAT0_FE         _HX8(10)        /* Bit 4: Framing Error */
#define Z16F_UARTSTAT0_BRKD       _HX8(08)        /* Bit 3: Break Detect */
#define Z16F_UARTSTAT0_TDRE       _HX8(04)        /* Bit 2: Transmitter Data Register Empty */
#define Z16F_UARTSTAT0_TXE        _HX8(02)        /* Bit 1: Transmitter Empty */
#define Z16F_UARTSTAT0_CTS        _HX8(01)        /* Bit 0: Clear To Send */

/* UART0/1 Control 0/1 Register Bit Definitions *************************************/

#define Z16F_UARTCTL0_TEN         _HX8(80)        /* Bit 7: Transmit Enable */
#define Z16F_UARTCTL0_REN         _HX8(40)        /* Bit 6: Receive Enable */
#define Z16F_UARTCTL0_CTSE        _HX8(20)        /* Bit 5: CTS Enable */
#define Z16F_UARTCTL0_PEN         _HX8(10)        /* Bit 4: Parity Enable */
#define Z16F_UARTCTL0_PSEL        _HX8(08)        /* Bit 3: Odd Parity Select */
#define Z16F_UARTCTL0_SBRK        _HX8(04)        /* Bit 2: Send Break */
#define Z16F_UARTCTL0_STOP        _HX8(02)        /* Bit 1: Stop Bit Select */
#define Z16F_UARTCTL0_LBEN        _HX8(01)        /* Bit 0: Loopback Enable */

#define Z16F_UARTCTL1_MPMD1       _HX8(80)        /* Bit 7: Multiprocessor Mode (bit1) */
#define Z16F_UARTCTL1_MPEN        _HX8(40)        /* Bit 6: Multiprocessor Enable */
#define Z16F_UARTCTL1_MPMD0       _HX8(20)        /* Bit 5: Multiprocessor Mode (bit0) */
#define Z16F_UARTCTL1_MPBT        _HX8(10)        /* Bit 4: Multiprocessor Bit Transmit */
#define Z16F_UARTCTL1_DEPOL       _HX8(08)        /* Bit 3: Driver Enable Polarity */
#define Z16F_UARTCTL1_BRGCTL      _HX8(04)        /* Bit 2: Baud Rate Generator Control */
#define Z16F_UARTCTL1_RDAIRQ      _HX8(02)        /* Bit 1: Receive Data Interrupt Enable */
#define Z16F_UARTCTL1_IREN        _HX8(01)        /* Bit 0: Infrared Encoder/Decoder Enable */

/* UART0/1 Mode Status/Select Register Bit Definitions ******************************/

#define Z16F_UARTMDSEL_NORMAL     _HX8(00)        /* Bits 5-7=0: Multiprocessor and Normal Mode */
#define Z16F_UARTMDSEL_FILTER     _HX8(20)        /* Bits 5-7=1: Noise Filter Control/Status */
#define Z16F_UARTMDSEL_LINP       _HX8(40)        /* Bits 5-7=2: LIN protocol Control/Status */
#define Z16F_UARTMDSEL_HWREV      _HX8(e0)        /* Bits 5-7=7: LIN-UART Hardware Revision */
                                                  /* Bits 0-4:   Mode dependent status */

/* ESPI registers *******************************************************************/

#define Z16F_ESPI_DATA            _HX32(ffffe260) /*  8-bit: ESPI Data */
#define Z16F_ESPI_DCR             _HX32(ffffe261) /*  8-bit: ESPI Transmit Data Command */
#define Z16F_ESPI_CTL             _HX32(ffffe262) /*  8-bit: ESPI Control */
#define Z16F_ESPI_MODE            _HX32(ffffe263) /*  8-bit: ESPI Mode */
#define Z16F_ESPI_STAT            _HX32(ffffe264) /*  8-bit: ESPI Status */
#define Z16F_ESPI_STATE           _HX32(ffffe265) /*  8-bit: ESPI State */
#define Z16F_ESPI_BR              _HX32(ffffe266) /*  16-bit: ESPI Baud Rate High Byte */
#  define Z16F_ESPI_BRH           _HX32(ffffe266) /*  8-bit: ESPI Baud Rate High Byte */
#  define Z16F_ESPI_BRL           _HX32(ffffe267) /*  8-bit: ESPI Baud Rate Low Byte */

/* ESPI register bit definitions ****************************************************/

#define Z16F_ESPI_DCR_SSV         _HX8(01)        /* Bit 0: Slave Select Value */
#define Z16F_ESPI_DCR_TEOF        _HX8(02)        /* Bit 1: Transmit End of Frame */

#define Z16F_ESPI_CTL_ESPIEN0     _HX8(01)        /* Bit 0: ESPI Enable and Direction Control */
#define Z16F_ESPI_CTL_MMEN        _HX8(02)        /* Bit 1: ESPI Master Mode Enable */
#define Z16F_ESPI_CTL_WOR         _HX8(04)        /* Bit 2: Wire-OR (Open-Drain) Mode Enabled */
#define Z16F_ESPI_CTL_CLKPOL      _HX8(08)        /* Bit 3: Clock Polarity */
#define Z16F_ESPI_CTL_PHASE       _HX8(10)        /* Bit 4: Phase Select */
#define Z16F_ESPI_CTL_BRGCTL      _HX8(20)        /* Bit 5: Baud Rate Generator Control */
#define Z16F_ESPI_CTL_ESPIEN1     _HX8(40)        /* Bit 6: ESPI Enable and Direction Control */
#define Z16F_ESPI_CTL_DIRQE       _HX8(80)        /* Bit 7: Data Interrupt Request Enable */

#define Z16F_ESPI_MODE_SSPO       _HX8(01)                                 /* Bit 0: Slave Select Polarity */
#define Z16F_ESPI_MODE_SSIO       _HX8(02)                                 /* Bit 1: Slave Select I/O */
#define Z16F_ESPI_MODE_NUMBITS_SHIFT   (2)                                 /* Bits 2-4: Number of Data Bits Per Character */
#define Z16F_ESPI_MODE_NUMBITS_MASK    (7 << Z16F_ESPI_MODE_NUMBITS_SHIFT)
#  define Z16F_ESPI_MODE_NUMBITS_8BITS (0 << Z16F_ESPI_MODE_NUMBITS_SHIFT) /* 8 bits */
#  define Z16F_ESPI_MODE_NUMBITS_1BIT  (1 << Z16F_ESPI_MODE_NUMBITS_SHIFT) /* 1 bit */
#  define Z16F_ESPI_MODE_NUMBITS_2BITS (2 << Z16F_ESPI_MODE_NUMBITS_SHIFT) /* 2 bits */
#  define Z16F_ESPI_MODE_NUMBITS_3BITS (3 << Z16F_ESPI_MODE_NUMBITS_SHIFT) /* 3 bits */
#  define Z16F_ESPI_MODE_NUMBITS_4BITS (4 << Z16F_ESPI_MODE_NUMBITS_SHIFT) /* 4 bits */
#  define Z16F_ESPI_MODE_NUMBITS_5BITS (5 << Z16F_ESPI_MODE_NUMBITS_SHIFT) /* 5 bits */
#  define Z16F_ESPI_MODE_NUMBITS_6BITS (6 << Z16F_ESPI_MODE_NUMBITS_SHIFT) /* 6 bits */
#  define Z16F_ESPI_MODE_NUMBITS_7BITS (7 << Z16F_ESPI_MODE_NUMBITS_SHIFT) /* 7 bits */
#define Z16F_ESPI_MODE_SSMD_SHIFT      (5)                                 /* Bits 5-7: SLAVE SELECT Mode */
#define Z16F_ESPI_MODE_SSMD_MASK       (7 << Z16F_ESPI_MODE_SSMD_SHIFT)
#  define Z16F_ESPI_MODE_SSMD_SPI      (0 << Z16F_ESPI_MODE_SSMD_SHIFT)    /* SPI mode */
#  define Z16F_ESPI_MODE_SSMD_LPBK     (1 << Z16F_ESPI_MODE_SSMD_SHIFT)    /* LOOPBACK Mode */
#  define Z16F_ESPI_MODE_SSMD_I2S      (2 << Z16F_ESPI_MODE_SSMD_SHIFT)    /* I2S Mode */

#define Z16F_ESPI_STAT_SLAS       _HX8(01)        /* Bit 0: Slave Select */
#define Z16F_ESPI_STAT_TFST       _HX8(02)        /* Bit 1: Transfer Status */
#define Z16F_ESPI_STAT_RDRF       _HX8(04)        /* Bit 2: Receive Data Register Full */
#define Z16F_ESPI_STAT_ROVR       _HX8(08)        /* Bit 3: Receive Overrun */
#define Z16F_ESPI_STAT_ABT        _HX8(10)        /* Bit 4: Slave mode transaction abort */
#define Z16F_ESPI_STAT_COL        _HX8(20)        /* Bit 5: Collision */
#define Z16F_ESPI_STAT_TUND       _HX8(40)        /* Bit 6: Transmit Underrun */
#define Z16F_ESPI_STAT_TDRE       _HX8(80)        /* Bit 7: Transmit Data Register Empty */

#define Z16F_ESPI_STATE_SHIFT     (0)                             /* Bits 0-5: ESPI State Machine */
#define Z16F_ESPI_STATE_MASK      (0x3f << Z16F_ESPI_STATE_SHIFT)
#  define Z16F_ESPI_STATE_IDLE    (0x00 << Z16F_ESPI_STATE_SHIFT) /* Idle */
#  define Z16F_ESPI_STATE_SWAIT   (0x01 << Z16F_ESPI_STATE_SHIFT) /* Slave Wait For SCK */
#  define Z16F_ESPI_STATE_I2SSD0  (0x02 << Z16F_ESPI_STATE_SHIFT) /* I2S slave mode start delay */
#  define Z16F_ESPI_STATE_I2SSD1  (0x03 << Z16F_ESPI_STATE_SHIFT) /* I2S slave mode start delay */
#  define Z16F_ESPI_STATE_SPIMD   (0x10 << Z16F_ESPI_STATE_SHIFT) /* SPI master mode start delay */
#  define Z16F_ESPI_STATE_I2SMD0  (0x31 << Z16F_ESPI_STATE_SHIFT) /* I2S master mode start delay */
#  define Z16F_ESPI_STATE_I2SMD1  (0x32 << Z16F_ESPI_STATE_SHIFT) /* I2S master mode start delay */
#  define Z16F_ESPI_STATE_B7RCV   (0x2e << Z16F_ESPI_STATE_SHIFT) /* Bit 7 Receive */
#  define Z16F_ESPI_STATE_B7XMT   (0x2f << Z16F_ESPI_STATE_SHIFT) /* Bit 7 Transmit */
#  define Z16F_ESPI_STATE_B6RCV   (0x2c << Z16F_ESPI_STATE_SHIFT) /* Bit 6 Receive */
#  define Z16F_ESPI_STATE_B6XMT   (0x2d << Z16F_ESPI_STATE_SHIFT) /* Bit 6 Transmit */
#  define Z16F_ESPI_STATE_B5RCV   (0x2a << Z16F_ESPI_STATE_SHIFT) /* Bit 5 Receive */
#  define Z16F_ESPI_STATE_B5XMT   (0x2b << Z16F_ESPI_STATE_SHIFT) /* Bit 5 Transmit */
#  define Z16F_ESPI_STATE_B4RCV   (0x28 << Z16F_ESPI_STATE_SHIFT) /* Bit 4 Receive */
#  define Z16F_ESPI_STATE_B4XMT   (0x29 << Z16F_ESPI_STATE_SHIFT) /* Bit 4 Transmit */
#  define Z16F_ESPI_STATE_B3RCV   (0x26 << Z16F_ESPI_STATE_SHIFT) /* Bit 3 Receive */
#  define Z16F_ESPI_STATE_B3XMT   (0x27 << Z16F_ESPI_STATE_SHIFT) /* Bit 3 Transmit */
#  define Z16F_ESPI_STATE_B2RCV   (0x24 << Z16F_ESPI_STATE_SHIFT) /* Bit 2 Receive */
#  define Z16F_ESPI_STATE_B2XMT   (0x25 << Z16F_ESPI_STATE_SHIFT) /* Bit 2 Transmit */
#  define Z16F_ESPI_STATE_B1RCV   (0x22 << Z16F_ESPI_STATE_SHIFT) /* Bit 1 Receive */
#  define Z16F_ESPI_STATE_B1XMT   (0x23 << Z16F_ESPI_STATE_SHIFT) /* Bit 1 Transmit */
#  define Z16F_ESPI_STATE_B0RCV   (0x20 << Z16F_ESPI_STATE_SHIFT) /* Bit 0 Receive */
#  define Z16F_ESPI_STATE_B0XMT   (0x21 << Z16F_ESPI_STATE_SHIFT) /* Bit 0 Transmit */
#define Z16F_ESPI_STATE_SDI       _HX8(40)                        /* Bit 6: Serial Data Input */
#define Z16F_ESPI_STATE_SCKI      _HX8(80)                        /* Bit 7: Serial Clock Input */

/* Timer0/1/2 registers *************************************************************/

#define Z16F_TIMER0_HL            _HX32(ffffe300) /* 16-bit: Timer 0 */
#  define Z16F_TIMER0_H           _HX32(ffffe300) /*  8-bit: Timer 0 High Byte */
#  define Z16F_TIMER0_L           _HX32(ffffe301) /*  8-bit: Timer 0 Low Byte */
#define Z16F_TIMER0_R             _HX32(ffffe302) /* 16-bit: Timer 0 Reload */
#  define Z16F_TIMER0_RH          _HX32(ffffe302) /*  8-bit: Timer 0 Reload High Byte */
# define Z16F_TIMER0_RL           _HX32(ffffe303) /*  8-bit: Timer 0 Reload Low Byte */
#define Z16F_TIMER0_PWM           _HX32(ffffe304) /* 16-bit: Timer 0 PWM */
#  define Z16F_TIMER0_PWMH        _HX32(ffffe304) /*  8-bit: Timer 0 PWM High Byte */
#  define Z16F_TIMER0_PWML        _HX32(ffffe305) /*  8-bit: Timer 0 PWM Low Byte */
#define Z16F_TIMER0_CTL           _HX32(ffffe306) /* 16-bit: Timer 0 Control */
#  define Z16F_TIMER0_CTL0        _HX32(ffffe306) /*  8-bit: Timer 0 Control 0 */
#  define Z16F_TIMER0_CTL1        _HX32(ffffe307) /*  8-bit: Timer 0 Control 1 */

#define Z16F_TIMER1_HL            _HX32(ffffe310) /* 16-bit: Timer 1 */
#  define Z16F_TIMER1_H           _HX32(ffffe310) /*  8-bit: Timer 1 High Byte */
#  define Z16F_TIMER1_L           _HX32(ffffe311) /*  8-bit: Timer 1 Low Byte */
#define Z16F_TIMER1_R             _HX32(ffffe312) /* 16-bit: Timer 1 Reload */
#  define Z16F_TIMER1_RH          _HX32(ffffe312) /*  8-bit: Timer 1 Reload High Byte */
#  define Z16F_TIMER1_RL          _HX32(ffffe313) /*  8-bit: Timer 1 Reload Low Byte */
#define Z16F_TIMER1_PWM           _HX32(ffffe314) /* 16-bit: Timer 1 PWM */
#  define Z16F_TIMER1_PWMH        _HX32(ffffe314) /*  8-bit: Timer 1 PWM High Byte */
#  define Z16F_TIMER1_PWML        _HX32(ffffe315) /*  8-bit: Timer 1 PWM Low Byte */
#define Z16F_TIMER1_CTL           _HX32(ffffe316) /* 16-bit: Timer 1 Control */
#  define Z16F_TIMER1_CTL0        _HX32(ffffe316) /*  8-bit: Timer 1 Control 0 */
#  define Z16F_TIMER1_CTL1        _HX32(ffffe317) /*  8-bit: Timer 1 Control 1 */

#define Z16F_TIMER2_HL            _HX32(ffffe320) /* 16-bit: Timer 2 */
#  define Z16F_TIMER2_H           _HX32(ffffe320) /*  8-bit: Timer 2 High Byte */
#  define Z16F_TIMER2_L           _HX32(ffffe321) /*  8-bit: Timer 2 Low Byte */
#define Z16F_TIMER2_R             _HX32(ffffe322) /* 16-bit: Timer 2 Reload */
#  define Z16F_TIMER2_RH          _HX32(ffffe322) /*  8-bit: Timer 2 Reload High Byte */
#  define Z16F_TIMER2_RL          _HX32(ffffe323) /*  8-bit: Timer 2 Reload Low Byte */
#define Z16F_TIMER2_PWM           _HX32(ffffe324) /* 16-bit: Timer 2 PWM */
#  define Z16F_TIMER2_PWMH        _HX32(ffffe324) /*  8-bit: Timer 2 PWM High Byte */
#  define Z16F_TIMER2_PWML        _HX32(ffffe325) /*  8-bit: Timer 2 PWM Low Byte */
#define Z16F_TIMER2_CTL           _HX32(ffffe326) /* 16-bit: Timer 2 Control */
#  define Z16F_TIMER2_CTL0        _HX32(ffffe326) /*  8-bit: Timer 2 Control 0 */
#  define Z16F_TIMER2_CTL1        _HX32(ffffe327) /*  8-bit: Timer 2 Control 1 */

/* Common timer0/1/2 register bit definitions ***************************************/

#define Z16F_TIMERCTL0_TMODE      _HX8(80)        /* Bit 7: Timer mode */
                                                  /* Bits 5-6: Timer configuration,
                                                   * Interpretation depends on timer mode */
#define Z16F_TIMERCTL0_RELOAD     _HX8(00)        /*   Interrupt occurs on reload or capture */
#define Z16F_TIMERCTL0_IDISABLED  _HX8(40)        /*   Disabled */
#define Z16F_TIMERCTL0_IINACTIVE  _HX8(40)        /*   Interrupt occurs on inactive gate edge */
#define Z16F_TIMERCTL0_ICAPTURE   _HX8(40)        /*   Interrupt occurs on capture */
#define Z16F_TIMERCTL0_IRELOAD    _HX8(60)        /*   Interrupt occurs on reload */
#define Z16F_TIMERCTL0_CASCADE    _HX8(10)        /* Bit 4: Timer is cascaded */
                                                  /* Bits 1-2: PW mode */
#define Z16F_TIMERCTL0_NODELAY    _HZ8(00)        /*    No delay */
#define Z16F_TIMERCTL0_DELAY2     _HZ8(01)        /*      2 cycle delay */
#define Z16F_TIMERCTL0_DELAY4     _HZ8(02)        /*      4 cycle delay */
#define Z16F_TIMERCTL0_DELAY8     _HZ8(03)        /*      8 cycle delay */
#define Z16F_TIMERCTL0_DELAY16    _HZ8(04)        /*     16 cycle delay */
#define Z16F_TIMERCTL0_DELAY32    _HZ8(05)        /*     32 cycle delay */
#define Z16F_TIMERCTL0_DELAY64    _HZ8(06)        /*     64 cycle delay */
#define Z16F_TIMERCTL0_DELAY128   _HZ8(07)        /*    128 cycle delay */

#define Z16F_TIMERCTL1_TEN        _HX8(80)        /* Bit 7: Timer enable */
#define Z16F_TIMERCTL1_TPOL       _HX8(40)        /* Bit 6: Input output polarity */
#define Z16F_TIMERSCTL1_DIVMASK   _HX8(38)        /* Bits 3-5: Timer prescale value */
#  define Z16F_TIMERSCTL1_DIV1    _HX8(00)        /*   Divide by   1 */
#  define Z16F_TIMERSCTL1_DIV2    _HX8(08)        /*   Divide by   2 */
#  define Z16F_TIMERSCTL1_DIV4    _HX8(10)        /*   Divide by   4 */
#  define Z16F_TIMERSCTL1_DIV8    _HX8(18)        /*   Divide by   8 */
#  define Z16F_TIMERSCTL1_DIV16   _HX8(20)        /*   Divide by  16 */
#  define Z16F_TIMERSCTL1_DIV32   _HX8(28)        /*   Divide by  32 */
#  define Z16F_TIMERSCTL1_DIV64   _HX8(30)        /*   Divide by  64 */
#  define Z16F_TIMERSCTL1_DIV128  _HX8(38)        /*   Divide by 128 */
#define Z16F_TIMERSCTL1_MODEMASK  _HX8(07)        /* Bits 0-2: Timer mode + CTL0 TMODE bit*/
#  define Z16F_TIMERSCTL1_ONESHOT _HX8(00)        /*   One shot mode (CTL0 TMOD = 0) */
#  define Z16F_TIMERSCTL1_PWMDO   _HX8(00)        /*   One shot mode (CTL0 TMOD = 1) */
#  define Z16F_TIMERSCTL1_CONT    _HX8(01)        /*   Continuous mode (CTL0 TMOD = 0)*/
#  define Z16F_TIMERSCTL1_CAPRST  _HX8(01)        /*   Capture restart mode (CTL0 TMOD = 1)*/
#  define Z16F_TIMERSCTL1_COUNTER _HX8(02)        /*   Counter mode (CTL0 TMOD = 0)*/
#  define Z16F_TIMERSCTL1_CMPCNTR _HX8(02)        /*   Comparator counter mode (CTL0 TMOD = 1)*/
#  define Z16F_TIMERSCTL1_PWMSO   _HX8(03)        /*   PWM single output mode (CTL0 TMOD = 0)*/
#  define Z16F_TIMERSCTL1_TRIGOS  _HX8(03)        /*   Triggered one shot (CTL0 TMOD = 1)*/
#  define Z16F_TIMERSCTL1_CAPTURE _HX8(04)        /*   Capture mode (CTL0 TMOD = 0)*/
#  define Z16F_TIMERSCTL1_COMPARE _HX8(05)        /*   Compare mode (CTL0 TMOD = 0)*/
#  define Z16F_TIMERSCTL1_GATED   _HX8(06)        /*   Gated mode (CTL0 TMOD = 0)*/
#  define Z16F_TIMERSCTL1_CAPCMP  _HX8(07)        /*   Capture/Compare mode (CTL0 TMOD = 0)*/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* The following two routines are called from the low-level reset logic.
 * z16f_board_initialize() must be provided by the board-specific logic;
 * z16f_lowuartinit() is called only if debugging support for z16_lowputc (or getc)
 * is enabled.
 */

void z16f_board_initialize(void);
#if defined(CONFIG_Z16_LOWPUTC) || defined(CONFIG_Z16_LOWGETC)
void z16f_lowuartinit(void);
#endif

/* This function handles Z16F system exceptions */

void z16f_sysexec(FAR chipreg_t *regs);

/* Entry point to reset the processor */

void z16f_reset(void);

/* The following must be provided by board-specific logic that uses the ZNeo
 * ESPI
 */

#ifdef CONFIG_Z16F_ESPI

struct spi_dev_s;  /* Forward reference */

/* Initialize the selected SPI port */

FAR struct spi_dev_s *z16_spibus_initialize(int port);

/* Select an SPI device (see include/nuttx/spi/spi.h) */

void z16f_espi_select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);

/* Provide SPI device status (see include/nuttx/spi/spi.h) */

uint8_t z16f_espi_status(FAR struct spi_dev_s *dev, uint32_t devid);

/* Select CMD/DATA options (often used with LCD devices) */

#ifdef CONFIG_SPI_CMDDATA
int z16f_espi_cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_Z16_SRC_Z16F_CHIP_H */

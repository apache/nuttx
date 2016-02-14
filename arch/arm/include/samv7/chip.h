/************************************************************************************
 * arch/arm/include/samv7/chip.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_SAMV7_CHIP_H
#define __ARCH_ARM_INCLUDE_SAMV7_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip */

/* SAME70Q19 -  512 Kbytes FLASH / 256 Kbytes SRAM
 * SAME70Q20 - 1024 Kbytes FLASH / 384 Kbytes SRAM
 * SAME70Q21 - 2048 Kbytes FLASH / 384 Kbytes SRAM
 *
 * LQFP144 and LFBGA144 packaging
 */

#if defined(CONFIG_ARCH_CHIP_SAME70Q19) || defined(CONFIG_ARCH_CHIP_SAME70Q20) || \
    defined(CONFIG_ARCH_CHIP_SAME70Q21)

/* Internal memory */

#if defined(CONFIG_ARCH_CHIP_SAME70Q19)
#  define SAMV7_FLASH_SIZE            (512*1024)    /*  512KB */
#  define SAMV7_SRAM_SIZE             (256*1024)    /*  256KB */
#elif defined(CONFIG_ARCH_CHIP_SAME70Q20)
#  define SAMV7_FLASH_SIZE            (1024*1024)   /* 1024KB */
#  define SAMV7_SRAM_SIZE             (384*1024)    /*  384KB */
#else /* if defined(CONFIG_ARCH_CHIP_SAME70Q21) */
#  define SAMV7_FLASH_SIZE            (2048*1024)   /* 2048KB */
#  define SAMV7_SRAM_SIZE             (384*1024)    /*  384KB */
#endif

#  define SAMV7_BSRAM_SIZE            (1*1024)      /*    1KB Backup SRAM */

/* Peripherals */

#  define SAMV7_NPIO                  5             /* 5 PIO ports A-E */
#  define SAMV7_NEBI                  1             /* Have External Bus Interface (EBI) */
#  define SAMV7_NSDRAMC               1             /* Have SDRAM controller (SDRAMC) */
#  define SAMV7_NMLB                  0             /* No MediaLB interface (MLB) */
#  define SAMV7_NDMACHAN              24            /* 24 Central DMA Channels */
#  define SAMV7_NADC12                24            /* 24 12-bit ADC channels */
#  define SAMV7_NDAC12                2             /* 2 12-bit DAC channels */
#  define SAMV7_NTCCH                 12            /* 12 Timer/counter channels */
#  define SAMV7_NTCCHIO               36            /* 36 Timer/counter channels I/O */
#  define SAMV7_NUSART                3             /* 3 USARTs */
#  define SAMV7_NUART                 5             /* 5 UARTs */
#  define SAMV7_NQSPI                 1             /* 1 Quad SPI */
#  define SAMV7_NSPI                  2             /* 2 SPI, SPI0-1 */
#  define SAMV7_NTWIHS                3             /* 3 TWIHS */
#  define SAMV7_NHSMCI4               1             /* 1 4-bit HSMCI port */
#  define SAMV7_NCAN                  2             /* 2 CAN ports */
#  define SAMV7_NEMAC                 1             /* 1 Ethernet MAC (GMAC) */
#  define SAMV7_NEMACMII              1             /* 1 Ethernet MAC MII interface */
#  define SAMV7_NEMACRMII             1             /* 1 Ethernet MAC RMII interface */
#  define SAMV7_NISI12                1             /* 1 12-bit ISI interface */
#  define SAMV7_NISI8                 0             /* No 8-bit ISI interface */
#  define SAMV7_NSSC                  1             /* 1 SSC */
#  define SAMV7_NUDPHS                1             /* 1 USB high speed device */
#  define SAMV7_NUHPHS                1             /* 1 USB high speed embedded Mini-Host */
#  define SAMV7_NUDPFS                0             /* No USB full speed device */
#  define SAMV7_NUHPFS                0             /* No USB full speed embedded host */
#  define SAMV7_NACC                  1             /* 1 Analog comparator */
#  define SAMV7_NETM                  1             /* 1 Embedded Trace Macrocell (ETM) */

/* SAME70N19 -  512 Kbytes FLASH / 256 Kbytes SRAM
 * SAME70N20 - 1024 Kbytes FLASH / 384 Kbytes SRAM
 * SAME70N21 - 2048 Kbytes FLASH / 384 Kbytes SRAM
 *
 * LQFP100 and TFBGA100 packaging
 */

#elif defined(CONFIG_ARCH_CHIP_SAME70N19) || defined(CONFIG_ARCH_CHIP_SAME70N20) || \
      defined(CONFIG_ARCH_CHIP_SAME70N21)

/* Internal memory */

#if defined(CONFIG_ARCH_CHIP_SAME70N19)
#  define SAMV7_FLASH_SIZE            (512*1024)    /*  512KB */
#  define SAMV7_SRAM_SIZE             (256*1024)    /*  256KB */
#elif defined(CONFIG_ARCH_CHIP_SAME70N20)
#  define SAMV7_FLASH_SIZE            (1024*1024)   /* 1024KB */
#  define SAMV7_SRAM_SIZE             (384*1024)    /*  384KB */
#else /* if defined(CONFIG_ARCH_CHIP_SAME70N21) */
#  define SAMV7_FLASH_SIZE            (2048*1024)   /* 2048KB */
#  define SAMV7_SRAM_SIZE             (384*1024)    /*  384KB */
#endif

#  define SAMV7_BSRAM_SIZE            (1*1024)      /*    1KB Backup SRAM */

/* Peripherals */

#  define SAMV7_NPIO                  5             /* 5 PIO ports A-E */
#  define SAMV7_NEBI                  0             /* No External Bus Interface (EBI) */
#  define SAMV7_NSDRAMC               0             /* No SDRAM controller (SDRAMC) */
#  define SAMV7_NMLB                  0             /* No MediaLB interface (MLB) */
#  define SAMV7_NDMACHAN              24            /* 24 Central DMA Channels */
#  define SAMV7_NADC12                10            /* 10 12-bit ADC channels */
#  define SAMV7_NDAC12                2             /* 2 12-bit DAC channels */
#  define SAMV7_NTCCH                 12            /* 12 Timer/counter channels */
#  define SAMV7_NTCCHIO               9             /* 9 Timer/counter channels I/O */
#  define SAMV7_NUSART                3             /* 3 USARTs */
#  define SAMV7_NUART                 5             /* 5 UARTs */
#  define SAMV7_NQSPI                 1             /* 1 Quad SPI */
#  define SAMV7_NSPI                  1             /* 1 SPI, SPI0 only */
#  define SAMV7_NTWIHS                3             /* 3 TWIHS */
#  define SAMV7_NHSMCI4               1             /* 1 4-bit HSMCI port */
#  define SAMV7_NCAN                  2             /* 2 CAN ports */
#  define SAMV7_NEMAC                 1             /* 1 Ethernet MAC (GMAC) */
#  define SAMV7_NEMACMII              1             /* 1 Ethernet MAC MII interface */
#  define SAMV7_NEMACRMII             1             /* 1 Ethernet MAC RMII interface */
#  define SAMV7_NISI12                1             /* 1 12-bit ISI interface */
#  define SAMV7_NISI8                 0             /* No 8-bit ISI interface */
#  define SAMV7_NSSC                  1             /* 1 SSC */
#  define SAMV7_NUDPHS                1             /* 1 USB high speed device */
#  define SAMV7_NUHPHS                1             /* 1 USB high speed embedded Mini-Host */
#  define SAMV7_NUDPFS                0             /* No USB full speed device */
#  define SAMV7_NUHPFS                0             /* No USB full speed embedded host */
#  define SAMV7_NACC                  1             /* 1 Analog comparator */
#  define SAMV7_NETM                  1             /* 1 Embedded Trace Macrocell (ETM) */

/* SAME70J19 -  512 Kbytes FLASH / 256 Kbytes SRAM
 * SAME70J20 - 1024 Kbytes FLASH / 384 Kbytes SRAM
 * SAME70J21 - 2048 Kbytes FLASH / 384 Kbytes SRAM
 *
 * LQFP64 and TFBGA64 packaging
 */

#elif defined(CONFIG_ARCH_CHIP_SAME70J19) || defined(CONFIG_ARCH_CHIP_SAME70J20) || \
      defined(CONFIG_ARCH_CHIP_SAME70J21)

/* Internal memory */

#if defined(CONFIG_ARCH_CHIP_SAME70J19)
#  define SAMV7_FLASH_SIZE            (512*1024)    /*  512KB */
#  define SAMV7_SRAM_SIZE             (256*1024)    /*  256KB */
#elif defined(CONFIG_ARCH_CHIP_SAME70J20)
#  define SAMV7_FLASH_SIZE            (1024*1024)   /* 1024KB */
#  define SAMV7_SRAM_SIZE             (384*1024)    /*  384KB */
#else /* if defined(CONFIG_ARCH_CHIP_SAME70J21) */
#  define SAMV7_FLASH_SIZE            (2048*1024)   /* 2048KB */
#  define SAMV7_SRAM_SIZE             (384*1024)    /*  384KB */
#endif

#  define SAMV7_BSRAM_SIZE            (1*1024)      /*    1KB Backup SRAM */

#  define SAMV7_NPIO                  5             /* 5 PIO ports A-E */
#  define SAMV7_NEBI                  0             /* No External Bus Interface (EBI) */
#  define SAMV7_NSDRAMC               0             /* No SDRAM controller (SDRAMC) */
#  define SAMV7_NMLB                  0             /* No MediaLB interface (MLB) */
#  define SAMV7_NDMACHAN              24            /* 24 Central DMA Channels */
#  define SAMV7_NADC12                5             /* 5 12-bit ADC channels */
#  define SAMV7_NDAC12                1             /* 1 12-bit DAC channel */
#  define SAMV7_NTCCH                 12            /* 12 Timer/counter channels */
#  define SAMV7_NTCCHIO               3             /* 3 Timer/counter channels I/O */
#  define SAMV7_NUSART                0             /* No USARTs */
#  define SAMV7_NUART                 5             /* 5 UARTs */
#  define SAMV7_NQSPI                 0             /* No Quad SPI */
#  define SAMV7_NSPI                  1             /* No SPI, QSPI functions in SPI mode only */
#  define SAMV7_NTWIHS                2             /* 2 TWIHS */
#  define SAMV7_NHSMCI4               0             /* No 4-bit HSMCI port */
#  define SAMV7_NCAN                  1             /* 1 CAN port */
#  define SAMV7_NEMAC                 1             /* 1 Ethernet MAC (GMAC) */
#  define SAMV7_NEMACMII              0             /* No Ethernet MAC MII interface */
#  define SAMV7_NEMACRMII             1             /* 1 Ethernet MAC RMII interface */
#  define SAMV7_NISI12                0             /* No 12-bit ISI interface */
#  define SAMV7_NISI8                 1             /* 1 8-bit ISI interface */
#  define SAMV7_NSSC                  1             /* 1 SSC */
#  define SAMV7_NUDPHS                0             /* No USB high speed device */
#  define SAMV7_NUHPHS                0             /* No USB high speed embedded Mini-Host */
#  define SAMV7_NUDPFS                1             /* 1 USB full speed device */
#  define SAMV7_NUHPFS                1             /* 1 USB full speed embedded host */
#  define SAMV7_NACC                  1             /* 1 Analog comparator */
#  define SAMV7_NETM                  1             /* 1 Embedded Trace Macrocell (ETM) */

/* SAMV71Q19 -  512 Kbytes FLASH / 256 Kbytes SRAM
 * SAMV71Q20 - 1024 Kbytes FLASH / 384 Kbytes SRAM
 * SAMV71Q21 - 2048 Kbytes FLASH / 384 Kbytes SRAM
 *
 * LQFP144 and LFBGA144 packaging
 */

#elif defined(CONFIG_ARCH_CHIP_SAMV71Q19) || defined(CONFIG_ARCH_CHIP_SAMV71Q20) || \
      defined(CONFIG_ARCH_CHIP_SAMV71Q21)

/* Internal memory */

#if defined(CONFIG_ARCH_CHIP_SAMV71Q19)
#  define SAMV7_FLASH_SIZE            (512*1024)    /*  512KB */
#  define SAMV7_SRAM_SIZE             (256*1024)    /*  256KB */
#elif defined(CONFIG_ARCH_CHIP_SAMV71Q20)
#  define SAMV7_FLASH_SIZE            (1024*1024)   /* 1024KB */
#  define SAMV7_SRAM_SIZE             (384*1024)    /*  384KB */
#else /* if defined(CONFIG_ARCH_CHIP_SAMV71Q21) */
#  define SAMV7_FLASH_SIZE            (2048*1024)   /* 2048KB */
#  define SAMV7_SRAM_SIZE             (384*1024)    /*  384KB */
#endif

#  define SAMV7_BSRAM_SIZE            (1*1024)      /*    1KB Backup SRAM */

/* Peripherals */

#  define SAMV7_NPIO                  5             /* 5 PIO ports A-E */
#  define SAMV7_NEBI                  1             /* Have External Bus Interface (EBI) */
#  define SAMV7_NSDRAMC               1             /* Have SDRAM controller (SDRAMC) */
#  define SAMV7_NMLB                  1             /* Have MediaLB interface (MLB) */
#  define SAMV7_NDMACHAN              24            /* 24 Central DMA Channels */
#  define SAMV7_NADC12                24            /* 24 12-bit ADC channels */
#  define SAMV7_NDAC12                2             /* 2 12-bit DAC channels */
#  define SAMV7_NTCCH                 12            /* 12 Timer/counter channels */
#  define SAMV7_NTCCHIO               36            /* 36 Timer/counter channels I/O */
#  define SAMV7_NUSART                3             /* 3 USARTs */
#  define SAMV7_NUART                 5             /* 5 UARTs */
#  define SAMV7_NQSPI                 1             /* 1 Quad SPI */
#  define SAMV7_NSPI                  2             /* 2 SPI, SPI0-1 */
#  define SAMV7_NTWIHS                3             /* 3 TWIHS */
#  define SAMV7_NHSMCI4               1             /* 1 4-bit HSMCI port */
#  define SAMV7_NCAN                  2             /* 2 CAN ports */
#  define SAMV7_NEMAC                 1             /* 1 Ethernet MAC (GMAC) */
#  define SAMV7_NEMACMII              1             /* 1 Ethernet MAC MII interface */
#  define SAMV7_NEMACRMII             1             /* 1 Ethernet MAC RMII interface */
#  define SAMV7_NISI12                1             /* 1 12-bit ISI interface */
#  define SAMV7_NISI8                 0             /* No 8-bit ISI interface */
#  define SAMV7_NSSC                  1             /* 1 SSC */
#  define SAMV7_NUDPHS                1             /* 1 USB high speed device */
#  define SAMV7_NUHPHS                1             /* 1 USB high speed embedded Mini-Host */
#  define SAMV7_NUDPFS                0             /* No USB full speed device */
#  define SAMV7_NUHPFS                0             /* No USB full speed embedded host */
#  define SAMV7_NACC                  1             /* 1 Analog comparator */
#  define SAMV7_NETM                  1             /* 1 Embedded Trace Macrocell (ETM) */

/* SAMV71N19 -  512 Kbytes FLASH / 256 Kbytes SRAM
 * SAMV71N20 - 1024 Kbytes FLASH / 384 Kbytes SRAM
 * SAMV71N21 - 2048 Kbytes FLASH / 384 Kbytes SRAM
 *
 * LQFP100 and TFBGA100 packaging
 */

#elif defined(CONFIG_ARCH_CHIP_SAMV71N19) || defined(CONFIG_ARCH_CHIP_SAMV71N20) || \
      defined(CONFIG_ARCH_CHIP_SAMV71N21)

/* Internal memory */

#if defined(CONFIG_ARCH_CHIP_SAMV71N19)
#  define SAMV7_FLASH_SIZE            (512*1024)    /*  512KB */
#  define SAMV7_SRAM_SIZE             (256*1024)    /*  256KB */
#elif defined(CONFIG_ARCH_CHIP_SAMV71N20)
#  define SAMV7_FLASH_SIZE            (1024*1024)   /* 1024KB */
#  define SAMV7_SRAM_SIZE             (384*1024)    /*  384KB */
#else /* if defined(CONFIG_ARCH_CHIP_SAMV71N21) */
#  define SAMV7_FLASH_SIZE            (2048*1024)   /* 2048KB */
#  define SAMV7_SRAM_SIZE             (384*1024)    /*  384KB */
#endif

#  define SAMV7_BSRAM_SIZE            (1*1024)      /*    1KB Backup SRAM */

/* Peripherals */

#  define SAMV7_NPIO                  5             /* 5 PIO ports A-E */
#  define SAMV7_NEBI                  0             /* No External Bus Interface (EBI) */
#  define SAMV7_NSDRAMC               0             /* No SDRAM controller (SDRAMC) */
#  define SAMV7_NMLB                  1             /* Have MediaLB interface (MLB) */
#  define SAMV7_NDMACHAN              24            /* 24 Central DMA Channels */
#  define SAMV7_NADC12                10            /* 10 12-bit ADC channels */
#  define SAMV7_NDAC12                2             /* 2 12-bit DAC channels */
#  define SAMV7_NTCCH                 12            /* 12 Timer/counter channels */
#  define SAMV7_NTCCHIO               9             /* 9 Timer/counter channels I/O */
#  define SAMV7_NUSART                3             /* 3 USARTs */
#  define SAMV7_NUART                 5             /* 5 UARTs */
#  define SAMV7_NQSPI                 1             /* 1 Quad SPI */
#  define SAMV7_NSPI                  1             /* 1 SPI, SPI0 */
#  define SAMV7_NTWIHS                3             /* 3 TWIHS */
#  define SAMV7_NHSMCI4               1             /* 1 4-bit HSMCI port */
#  define SAMV7_NCAN                  2             /* 2 CAN ports */
#  define SAMV7_NEMAC                 1             /* 1 Ethernet MAC (GMAC) */
#  define SAMV7_NEMACMII              1             /* 1 Ethernet MAC MII interface */
#  define SAMV7_NEMACRMII             1             /* 1 Ethernet MAC RMII interface */
#  define SAMV7_NISI12                1             /* 1 12-bit ISI interface */
#  define SAMV7_NISI8                 0             /* No 8-bit ISI interface */
#  define SAMV7_NSSC                  1             /* 1 SSC */
#  define SAMV7_NUDPHS                1             /* 1 USB high speed device */
#  define SAMV7_NUHPHS                1             /* 1 USB high speed embedded Mini-Host */
#  define SAMV7_NUDPFS                0             /* No USB full speed device */
#  define SAMV7_NUHPFS                0             /* No USB full speed embedded host */
#  define SAMV7_NACC                  1             /* 1 Analog comparator */
#  define SAMV7_NETM                  1             /* 1 Embedded Trace Macrocell (ETM) */

/* SAMV71J19 -  512 Kbytes FLASH / 256 Kbytes SRAM
 * SAMV71J20 - 1024 Kbytes FLASH / 384 Kbytes SRAM
 * SAMV71J21 - 2048 Kbytes FLASH / 384 Kbytes SRAM
 *
 * LQFP64, TFBGA64, and QFN64 packaging
 */

#elif defined(CONFIG_ARCH_CHIP_SAMV71J19) || defined(CONFIG_ARCH_CHIP_SAMV71J20) || \
      defined(CONFIG_ARCH_CHIP_SAMV71J21)

/* Internal memory */

#if defined(CONFIG_ARCH_CHIP_SAMV71J19)
#  define SAMV7_FLASH_SIZE            (512*1024)    /*  512KB */
#  define SAMV7_SRAM_SIZE             (256*1024)    /*  256KB */
#elif defined(CONFIG_ARCH_CHIP_SAMV71J20)
#  define SAMV7_FLASH_SIZE            (1024*1024)   /* 1024KB */
#  define SAMV7_SRAM_SIZE             (384*1024)    /*  384KB */
#else /* if defined(CONFIG_ARCH_CHIP_SAMV71J21) */
#  define SAMV7_FLASH_SIZE            (2048*1024)   /* 2048KB */
#  define SAMV7_SRAM_SIZE             (384*1024)    /*  384KB */
#endif

#define SAMV7_BSRAM_SIZE              (1*1024)      /*    1KB Backup SRAM */

/* Peripherals */

#  define SAMV7_NPIO                  5             /* 5 PIO ports A-E */
#  define SAMV7_NEBI                  0             /* No External Bus Interface (EBI) */
#  define SAMV7_NSDRAMC               0             /* No SDRAM controller (SDRAMC) */
#  define SAMV7_NMLB                  1             /* Have MediaLB interface (MLB) */
#  define SAMV7_NDMACHAN              24            /* 24 Central DMA Channels */
#  define SAMV7_NADC12                5             /* 5 12-bit ADC channels */
#  define SAMV7_NDAC12                1             /* 1 12-bit DAC channels */
#  define SAMV7_NTCCH                 12            /* 12 Timer/counter channels */
#  define SAMV7_NTCCHIO               3             /* 3 Timer/counter channels I/O */
#  define SAMV7_NUSART                0             /* No USARTs */
#  define SAMV7_NUART                 5             /* 5 UARTs */
#  define SAMV7_NQSPI                 0             /* No Quad SPI */
#  define SAMV7_NSPI                  1             /* 1 SPI, QSPI functions in SPI mode only */
#  define SAMV7_NTWIHS                2             /* 2 TWIHS */
#  define SAMV7_NHSMCI4               0             /* No 4-bit HSMCI port */
#  define SAMV7_NCAN                  1             /* 1 CAN port */
#  define SAMV7_NEMAC                 1             /* 1 Ethernet MAC (GMAC) */
#  define SAMV7_NEMACMII              0             /* No Ethernet MAC MII interface */
#  define SAMV7_NEMACRMII             1             /* 1 Ethernet MAC RMII interface */
#  define SAMV7_NISI12                0             /* No 12-bit ISI interface */
#  define SAMV7_NISI8                 1             /* 1 8-bit ISI interface */
#  define SAMV7_NSSC                  1             /* 1 SSC */
#  define SAMV7_NUDPHS                0             /* No USB high speed device */
#  define SAMV7_NUHPHS                0             /* No USB high speed embedded Mini-Host */
#  define SAMV7_NUDPFS                1             /* 1 USB full speed device */
#  define SAMV7_NUHPFS                1             /* 1 USB full speed embedded host */
#  define SAMV7_NACC                  1             /* 1 Analog comparator */
#  define SAMV7_NETM                  1             /* 1 Embedded Trace Macrocell (ETM) */

#else
#  error "Unknown SAMV7 chip type"
#endif

/* NVIC priority levels *************************************************************/
/* Each priority field holds a priority value, 0-15. The lower the value, the greater
 * the priority of the corresponding interrupt. The processor implements only
 * bits[7:6] of each field, bits[5:0] read as zero and ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN        0xc0 /* All bits[7:6] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT    0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX        0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP       0x40 /* Two bits of interrupt priority used */

/* If CONFIG_ARMV7M_USEBASEPRI is selected, then interrupts will be disabled
 * by setting the BASEPRI register to NVIC_SYSH_DISABLE_PRIORITY so that most
 * interrupts will not have execution priority.  SVCall must have execution
 * priority in all cases.
 *
 * In the normal cases, interrupts are not nest-able and all interrupts run
 * at an execution priority between NVIC_SYSH_PRIORITY_MIN and
 * NVIC_SYSH_PRIORITY_MAX (with NVIC_SYSH_PRIORITY_MAX reserved for SVCall).
 *
 * If, in addition, CONFIG_ARCH_HIPRI_INTERRUPT is defined, then special
 * high priority interrupts are supported.  These are not "nested" in the
 * normal sense of the word.  These high priority interrupts can interrupt
 * normal processing but execute outside of OS (although they can "get back
 * into the game" via a PendSV interrupt).
 *
 * In the normal course of things, interrupts must occasionally be disabled
 * using the up_irq_save() inline function to prevent contention in use of
 * resources that may be shared between interrupt level and non-interrupt
 * level logic.  Now the question arises, if CONFIG_ARCH_HIPRI_INTERRUPT,
 * do we disable all interrupts (except SVCall), or do we only disable the
 * "normal" interrupts.  Since the high priority interrupts cannot interact
 * with the OS, you may want to permit the high priority interrupts even if
 * interrupts are disabled.  The setting CONFIG_ARCH_INT_DISABLEALL can be
 * used to select either behavior:
 *
 *   ----------------------------+--------------+----------------------------
 *   CONFIG_ARCH_HIPRI_INTERRUPT |      NO      |             YES
 *   ----------------------------+--------------+--------------+-------------
 *   CONFIG_ARCH_INT_DISABLEALL  |     N/A      |     YES      |      NO
 *   ----------------------------+--------------+--------------+-------------
 *                               |              |              |    SVCall
 *                               |    SVCall    |    SVCall    |    HIGH
 *   Disable here and below --------> MAXNORMAL ---> HIGH --------> MAXNORMAL
 *                               |              |    MAXNORMAL |
 *   ----------------------------+--------------+--------------+-------------
 */

#if defined(CONFIG_ARCH_HIPRI_INTERRUPT) && defined(CONFIG_ARCH_INT_DISABLEALL)
#  define NVIC_SYSH_MAXNORMAL_PRIORITY  (NVIC_SYSH_PRIORITY_MAX + 2*NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_HIGH_PRIORITY       (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_DISABLE_PRIORITY    NVIC_SYSH_HIGH_PRIORITY
#  define NVIC_SYSH_SVCALL_PRIORITY     NVIC_SYSH_PRIORITY_MAX
#else
#  define NVIC_SYSH_MAXNORMAL_PRIORITY  (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_HIGH_PRIORITY       NVIC_SYSH_PRIORITY_MAX
#  define NVIC_SYSH_DISABLE_PRIORITY    NVIC_SYSH_MAXNORMAL_PRIORITY
#  define NVIC_SYSH_SVCALL_PRIORITY     NVIC_SYSH_PRIORITY_MAX
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_SAMV7_CHIP_H */

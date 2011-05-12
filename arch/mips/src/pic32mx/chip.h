/****************************************************************************
 * arch/mips/src/pic32mx/chip.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __ARCH_MIPS_SRC_PIC32MX_CHIP_H
#define __ARCH_MIPS_SRC_PIC32MX_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#if defined(CONFIG_ARCH_CHIP_PIC32MX320F032H)
#  define CHIP_PIC32MX3     1
#  undef  CHIP_PIC32MX4
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          40
#  define CHIP_BOOTFLASH_KB 12
#  define CHIP_PROGFLASH_KB 32
#  define CHIP_DATAMEM_KB   8
#  define CHIP_NTIMERS      5
#  define CHIP_NCAPTURE     5
#  define CHIP_NCOMPARE     5
#  define CHIP_NDMACH       0
#  define CHIP_NUSBDMACHAN  0
#  define CHIP_VREG
#  undef  CHIP_TRACE
#  define CHIP_NEUARTS      2
#  define CHIP_UARTFIFOD    4
#  define CHIP_NSPI         2
#  define CHIP_NI2C         2
#  define CHIP_NADC10       16
#  define CHIP_NCOMPARATORS 2
#  define CHIP_PMP
#  define CHIP_PSP
#  define CHIP_JTAH
#elif defined(CONFIG_ARCH_CHIP_PIC32MX320F064H)
#  define CHIP_PIC32MX3     1
#  undef  CHIP_PIC32MX4
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          80
#  define CHIP_BOOTFLASH_KB 12
#  define CHIP_PROGFLASH_KB 64
#  define CHIP_DATAMEM_KB   16
#  define CHIP_NTIMERS      5
#  define CHIP_NCAPTURE     5
#  define CHIP_NCOMPARE     5
#  define CHIP_NDMACH       0
#  define CHIP_NUSBDMACHAN  0
#  define CHIP_VREG
#  undef  CHIP_TRACE
#  define CHIP_NEUARTS      2
#  define CHIP_UARTFIFOD    4
#  define CHIP_NSPI         2
#  define CHIP_NI2C         2
#  define CHIP_NADC10       16
#  define CHIP_NCOMPARATORS 2
#  define CHIP_PMP
#  define CHIP_PSP
#  define CHIP_JTAH
#elif defined(CONFIG_ARCH_CHIP_PIC32MX320F128H)
#  define CHIP_PIC32MX3     1
#  undef  CHIP_PIC32MX4
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          80
#  define CHIP_BOOTFLASH_KB 12
#  define CHIP_PROGFLASH_KB 128
#  define CHIP_DATAMEM_KB   16
#  define CHIP_NTIMERS      5
#  define CHIP_NCAPTURE     5
#  define CHIP_NCOMPARE     5
#  define CHIP_NDMACH       0
#  define CHIP_NUSBDMACHAN  0
#  define CHIP_VREG
#  undef  CHIP_TRACE
#  define CHIP_NEUARTS      2
#  define CHIP_UARTFIFOD    4
#  define CHIP_NSPI         2
#  define CHIP_NI2C         2
#  define CHIP_NADC10       16
#  define CHIP_NCOMPARATORS 2
#  define CHIP_PMP
#  define CHIP_PSP
#  define CHIP_JTAH
#elif defined(CONFIG_ARCH_CHIP_PIC32MX340F128H)
#  define CHIP_PIC32MX3     1
#  undef  CHIP_PIC32MX4
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          80
#  define CHIP_BOOTFLASH_KB 12
#  define CHIP_PROGFLASH_KB 128
#  define CHIP_DATAMEM_KB   32
#  define CHIP_NTIMERS      5
#  define CHIP_NCAPTURE     5
#  define CHIP_NCOMPARE     5
#  define CHIP_NDMACH       4
#  define CHIP_NUSBDMACHAN  0
#  define CHIP_VREG
#  undef  CHIP_TRACE
#  define CHIP_NEUARTS      2
#  define CHIP_UARTFIFOD    4
#  define CHIP_NSPI         2
#  define CHIP_NI2C         2
#  define CHIP_NADC10       16
#  define CHIP_NCOMPARATORS 2
#  define CHIP_PMP
#  define CHIP_PSP
#  define CHIP_JTAH
#elif defined(CONFIG_ARCH_CHIP_PIC32MX340F256H)
#  define CHIP_PIC32MX3     1
#  undef  CHIP_PIC32MX4
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          80
#  define CHIP_BOOTFLASH_KB 12
#  define CHIP_PROGFLASH_KB 256
#  define CHIP_DATAMEM_KB   32
#  define CHIP_NTIMERS      5
#  define CHIP_NCAPTURE     5
#  define CHIP_NCOMPARE     5
#  define CHIP_NDMACH       4
#  define CHIP_NUSBDMACHAN  0
#  define CHIP_VREG
#  undef  CHIP_TRACE
#  define CHIP_NEUARTS      2
#  define CHIP_UARTFIFOD    4
#  define CHIP_NSPI         2
#  define CHIP_NI2C         2
#  define CHIP_NADC10       16
#  define CHIP_NCOMPARATORS 2
#  define CHIP_PMP
#  define CHIP_PSP
#  define CHIP_JTAH
#elif defined(CONFIG_ARCH_CHIP_PIC32MX340F512H)
#  define CHIP_PIC32MX3     1
#  undef  CHIP_PIC32MX4
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          80
#  define CHIP_BOOTFLASH_KB 12
#  define CHIP_PROGFLASH_KB 512
#  define CHIP_DATAMEM_KB   32
#  define CHIP_NTIMERS      5
#  define CHIP_NCAPTURE     5
#  define CHIP_NCOMPARE     5
#  define CHIP_NDMACH       4
#  define CHIP_NUSBDMACHAN  0
#  define CHIP_VREG
#  undef  CHIP_TRACE
#  define CHIP_NEUARTS      2
#  define CHIP_UARTFIFOD    4
#  define CHIP_NSPI         2
#  define CHIP_NI2C         2
#  define CHIP_NADC10       16
#  define CHIP_NCOMPARATORS 2
#  define CHIP_PMP
#  define CHIP_PSP
#  define CHIP_JTAH
#elif defined(CONFIG_ARCH_CHIP_PIC32MX320F128L)
#  define CHIP_PIC32MX3     1
#  undef  CHIP_PIC32MX4
#  define CHIP_NPINS        100 /* Package PT=100 BG=121 */
#  define CHIP_MHZ          80
#  define CHIP_BOOTFLASH_KB 12
#  define CHIP_PROGFLASH_KB 128
#  define CHIP_DATAMEM_KB   16
#  define CHIP_NTIMERS      5
#  define CHIP_NCAPTURE     5
#  define CHIP_NCOMPARE     5
#  define CHIP_NDMACH       0
#  define CHIP_NUSBDMACHAN  0
#  define CHIP_VREG
#  undef  CHIP_TRACE
#  define CHIP_NEUARTS      2
#  define CHIP_UARTFIFOD    4
#  define CHIP_NSPI         2
#  define CHIP_NI2C         2
#  define CHIP_NADC10       16
#  define CHIP_NCOMPARATORS 2
#  define CHIP_PMP
#  define CHIP_PSP
#  define CHIP_JTAH
#elif defined(CONFIG_ARCH_CHIP_PIC32MX340F128L)
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  define CHIP_NPINS        100 /* Package PT=100 BG=121 */
#  define CHIP_MHZ          80
#  define CHIP_BOOTFLASH_KB 12
#  define CHIP_PROGFLASH_KB 128
#  define CHIP_DATAMEM_KB   32
#  define CHIP_NTIMERS      5
#  define CHIP_NCAPTURE     5
#  define CHIP_NCOMPARE     5
#  define CHIP_NDMACH       4
#  define CHIP_NUSBDMACHAN  0
#  define CHIP_VREG
#  undef  CHIP_TRACE
#  define CHIP_NEUARTS      2
#  define CHIP_UARTFIFOD    4
#  define CHIP_NSPI         2
#  define CHIP_NI2C         2
#  define CHIP_NADC10       16
#  define CHIP_NCOMPARATORS 2
#  define CHIP_PMP
#  define CHIP_PSP
#  define CHIP_JTAH
#elif defined(CONFIG_ARCH_CHIP_PIC32MX360F256L)
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  define CHIP_NPINS        100 /* Package PT=100 BG=121 */
#  define CHIP_MHZ          80
#  define CHIP_BOOTFLASH_KB 12
#  define CHIP_PROGFLASH_KB 256
#  define CHIP_DATAMEM_KB   32
#  define CHIP_NTIMERS      5
#  define CHIP_NCAPTURE     5
#  define CHIP_NCOMPARE     5
#  define CHIP_NDMACH       4
#  define CHIP_NUSBDMACHAN  0
#  define CHIP_VREG
#  define CHIP_TRACE
#  define CHIP_NEUARTS      2
#  define CHIP_UARTFIFOD    4
#  define CHIP_NSPI         2
#  define CHIP_NI2C         2
#  define CHIP_NADC10       16
#  define CHIP_NCOMPARATORS 2
#  define CHIP_PMP
#  define CHIP_PSP
#  define CHIP_JTAH
#elif defined(CONFIG_ARCH_CHIP_PIC32MX360F512L)
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  define CHIP_NPINS        100 /* Package PT=100 BG=121 */
#  define CHIP_MHZ          80
#  define CHIP_BOOTFLASH_KB 12
#  define CHIP_PROGFLASH_KB 512
#  define CHIP_DATAMEM_KB   32
#  define CHIP_NTIMERS      5
#  define CHIP_NCAPTURE     5
#  define CHIP_NCOMPARE     5
#  define CHIP_NDMACH       4
#  define CHIP_NUSBDMACHAN  0
#  define CHIP_VREG
#  define CHIP_TRACE
#  define CHIP_NEUARTS      2
#  define CHIP_UARTFIFOD    4
#  define CHIP_NSPI         2
#  define CHIP_NI2C         2
#  define CHIP_NADC10       16
#  define CHIP_NCOMPARATORS 2
#  define CHIP_PMP
#  define CHIP_PSP
#  define CHIP_JTAH
#elif defined(CONFIG_ARCH_CHIP_PIC32MX420F032H)
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          40
#  define CHIP_BOOTFLASH_KB 12
#  define CHIP_PROGFLASH_KB 32
#  define CHIP_DATAMEM_KB   8
#  define CHIP_NTIMERS      5
#  define CHIP_NCAPTURE     5
#  define CHIP_NCOMPARE     5
#  define CHIP_NDMACH       0
#  define CHIP_NUSBDMACHAN  2
#  define CHIP_VREG
#  undef  CHIP_TRACE
#  define CHIP_NEUARTS      2
#  define CHIP_UARTFIFOD    4
#  define CHIP_NSPI         1
#  define CHIP_NI2C         2
#  define CHIP_NADC10       16
#  define CHIP_NCOMPARATORS 2
#  define CHIP_PMP
#  define CHIP_PSP
#  define CHIP_JTAH
#elif defined(CONFIG_ARCH_CHIP_PIC32MX440F128H)
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          40
#  define CHIP_BOOTFLASH_KB 12
#  define CHIP_PROGFLASH_KB 128
#  define CHIP_DATAMEM_KB   32
#  define CHIP_NTIMERS      5
#  define CHIP_NCAPTURE     5
#  define CHIP_NCOMPARE     5
#  define CHIP_NDMACH       4
#  define CHIP_NUSBDMACHAN  2
#  define CHIP_VREG
#  undef  CHIP_TRACE
#  define CHIP_NEUARTS      2
#  define CHIP_UARTFIFOD    4
#  define CHIP_NSPI         1
#  define CHIP_NI2C         2
#  define CHIP_NADC10       16
#  define CHIP_NCOMPARATORS 2
#  define CHIP_PMP
#  define CHIP_PSP
#  define CHIP_JTAH
#elif defined(CONFIG_ARCH_CHIP_PIC32MX440F256H)
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          80
#  define CHIP_BOOTFLASH_KB 12
#  define CHIP_PROGFLASH_KB 256
#  define CHIP_DATAMEM_KB   32
#  define CHIP_NTIMERS      5
#  define CHIP_NCAPTURE     5
#  define CHIP_NCOMPARE     5
#  define CHIP_NDMACH       4
#  define CHIP_NUSBDMACHAN  2
#  define CHIP_VREG
#  undef  CHIP_TRACE
#  define CHIP_NEUARTS      2
#  define CHIP_UARTFIFOD    4
#  define CHIP_NSPI         1
#  define CHIP_NI2C         2
#  define CHIP_NADC10       16
#  define CHIP_NCOMPARATORS 2
#  define CHIP_PMP
#  define CHIP_PSP
#  define CHIP_JTAH
#elif defined(CONFIG_ARCH_CHIP_PIC32MX440F512H)
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  define CHIP_NPINS        64  /* Package PT, MR */
#  define CHIP_MHZ          80
#  define CHIP_BOOTFLASH_KB 12
#  define CHIP_PROGFLASH_KB 512
#  define CHIP_DATAMEM_KB   32
#  define CHIP_NTIMERS      5
#  define CHIP_NCAPTURE     5
#  define CHIP_NCOMPARE     5
#  define CHIP_NDMACH       4
#  define CHIP_NUSBDMACHAN  2
#  define CHIP_VREG
#  undef  CHIP_TRACE
#  define CHIP_NEUARTS      2
#  define CHIP_UARTFIFOD    4
#  define CHIP_NSPI         1
#  define CHIP_NI2C         2
#  define CHIP_NADC10       16
#  define CHIP_NCOMPARATORS 2
#  define CHIP_PMP
#  define CHIP_PSP
#  define CHIP_JTAH
#elif defined(CONFIG_ARCH_CHIP_PIC32MX440F128L)
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  define CHIP_NPINS        100 /* Package PT=100 BG=121 */
#  define CHIP_MHZ          80
#  define CHIP_BOOTFLASH_KB 12
#  define CHIP_PROGFLASH_KB 128
#  define CHIP_DATAMEM_KB   32
#  define CHIP_NTIMERS      5
#  define CHIP_NCAPTURE     5
#  define CHIP_NCOMPARE     5
#  define CHIP_NDMACH       4
#  define CHIP_NUSBDMACHAN  2
#  define CHIP_VREG
#  undef  CHIP_TRACE
#  define CHIP_NEUARTS      2
#  define CHIP_UARTFIFOD    4
#  define CHIP_NSPI         2
#  define CHIP_NI2C         2
#  define CHIP_NADC10       16
#  define CHIP_NCOMPARATORS 2
#  define CHIP_PMP
#  define CHIP_PSP
#  define CHIP_JTAH
#elif defined(CONFIG_ARCH_CHIP_PIC32MX460F256L)
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  define CHIP_NPINS        100 /* Package PT=100 BG=121 */
#  define CHIP_MHZ          80
#  define CHIP_BOOTFLASH_KB 12
#  define CHIP_PROGFLASH_KB 256
#  define CHIP_DATAMEM_KB   32
#  define CHIP_NTIMERS      5
#  define CHIP_NCAPTURE     5
#  define CHIP_NCOMPARE     5
#  define CHIP_NDMACH       4
#  define CHIP_NUSBDMACHAN  2
#  define CHIP_VREG
#  define CHIP_TRACE
#  define CHIP_NEUARTS      2
#  define CHIP_UARTFIFOD    4
#  define CHIP_NSPI         2
#  define CHIP_NI2C         2
#  define CHIP_NADC10       16
#  define CHIP_NCOMPARATORS 2
#  define CHIP_PMP
#  define CHIP_PSP
#  define CHIP_JTAH
#elif defined(CONFIG_ARCH_CHIP_PIC32MX460F512L)
#  undef  CHIP_PIC32MX3
#  define CHIP_PIC32MX4     1
#  define CHIP_NPINS        100 /* Package PT=100 BG=121 */
#  define CHIP_MHZ          80
#  define CHIP_BOOTFLASH_KB 12
#  define CHIP_PROGFLASH_KB 512
#  define CHIP_DATAMEM_KB   32
#  define CHIP_NTIMERS      5
#  define CHIP_NCAPTURE     5
#  define CHIP_NCOMPARE     5
#  define CHIP_NDMACH       4
#  define CHIP_NUSBDMACHAN  2
#  define CHIP_VREG
#  define CHIP_TRACE
#  define CHIP_NEUARTS      2
#  define CHIP_UARTFIFOD    4
#  define CHIP_NSPI         2
#  define CHIP_NI2C         2
#  define CHIP_NADC10       16
#  define CHIP_NCOMPARATORS 2
#  define CHIP_PMP
#  define CHIP_PSP
#  define CHIP_JTAH
#else
#  error "Unrecognized PIC32 device
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_CHIP_H */

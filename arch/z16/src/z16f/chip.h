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

/************************************************************************************
 * Definitions
 ************************************************************************************/
 
/* Z16F Chip Variants */

#if defined(CONFIG_ARCH_CHIP_Z16F2810)
# define Z16F_INVMEM_SIZE (128*1024)
# define Z16F_IRAM_SIZE   (4*1024)
# undef  Z16F_HAVE_EXTMEM
#elif defined(CONFIG_ARCH_CHIP_Z16F2811)
# define Z16F_INVMEM_SIZE (128*1024)
# define Z16F_IRAM_SIZE   (4*1024)
# define Z16F_HAVE_EXTMEM  1
#elif defined(CONFIG_ARCH_CHIP_Z16F3211)
# define Z16F_INVMEM_SIZE (32*1024)
# define Z16F_IRAM_SIZE   (2*1024)
# define Z16F_HAVE_EXTMEM  1
#elif defined(CONFIG_ARCH_CHIP_Z16F6411)
# define Z16F_INVMEM_SIZE (64*1024)
# define Z16F_IRAM_SIZE   (4*1024)
# define Z16F_HAVE_EXTMEM  1
#else
# error "Z16F chip variant not specified"
#endif

/* Memory areas
 *
 * Internal non-volatile memory starts at address zero.  The size
 * of the internal non-volatile memory is chip-dependent.
 */
 
#define Z16F_INVMEM_BASE       0x000000

/* Most chip variants support external memory */

#ifdef Z16F_HAVE_EXTMEM
#  define Z16F_EXTMEMCS0_BASE  0x020000 /* External memory at CS0 */
#  define Z16F_EXTMEMCS0_SIZE  0x7e0000 /*   (actual depends on board) */
#  define Z16F_EXTMEMCS1_BASE  0x800000 /* External memory at CS1 */
#  define Z16F_EXTMEMCS1_SIZE  0x700000 /*   (actual depends on board) */
#  define Z16F_EXTMEMCS2A_BASE 0xf00000 /* External memory at CS2 */
#  define Z16F_EXTMEMCS2A_SIZE 0x0f8000 /*   (actual depends on board) */
#  define Z16F_EXTMEMCS2B_BASE 0xffc000 /* External memory at CS2 */
#  define Z16F_EXTMEMCS2B_SIZE 0x000800 /*   (actual depends on board) */
#endif

/* Internal RAM always ends at 0xffbfff.  The IRAM base address depends
 * on the size of the IRAM supported by the chip.
 */
 
#define Z16F_IRAM_BASE         (0xffc000 - Z16F_IRAM_SIZE)

/* External memory mapped peripherals, internal I/O memory and SFRS */

#define Z16F_EXTIO_BASE        0xffc800 /* External peripherals CS3-5 */
#define Z16F_EXTIO_SIZE        0x001800
#define Z16F_IIO_BASE          0xffe000 /* Internal I/O memory and SFRs */
#define Z16F_IIO_SIZE          0x001fff
 
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

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif  /* __Z16F_CHIP_H */

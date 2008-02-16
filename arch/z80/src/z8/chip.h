/************************************************************************************
 * arch/z80/src/z8/chip.h
 * arch/z80/src/chip/chip.h
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

#ifndef __Z8_CHIP_H
#define __Z8_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Hexadecimal Representation *******************************************************/

#ifdef __ASSEMBLY__
# define _HX(h)   %##h
#else
# define _HX(h)   0x##h
#endif

/* Memory Map
 *
 * 64Kb Program Memory (64K series)
 *  C:0000 - C:0001 : Flash options
 *  C:0002 - C:0037 : Vectors
 *                  : ROM data
 *                  : Code
 *
 * 4Kb Register File (64K series)
 *  R:020 - R:0ff   : 224 byte RDATA
 *  R:0e0 - R:0ef   :  16-byte working register area (RDATA)
 *  E:100 - E:eff   : 3.5 Kbyte EDATA
 *    f00 -   fff   : 256 byte control register area
 */

/* Special Function Registers *******************************************************
 *
 * Because of the many different ez80 configurations, we will rely on the
 * ZDS-II header file, ez8.h, to provide the correct addresses for each register.
 */

/* Timer Register Bit Definitions ***************************************************/

/* Timer control register */

#define Z8_TIMERCTL_TEN      _HX(80) /* Bit 7: Timer enabled */
#define Z8_TIMERCTL_TPOL     _HX(40) /* Bit 6: Timer input/output polarity */
#define Z8_TIMERCTL_DIV1     _HX(00) /* Bits 3-5: Pre-scale divisor */
#define Z8_TIMERCTL_DIV2     _HX(08)
#define Z8_TIMERCTL_DIV4     _HX(10)
#define Z8_TIMERCTL_DIV8     _HX(18)
#define Z8_TIMERCTL_DIV16    _HX(20)
#define Z8_TIMERCTL_DIV32    _HX(28)
#define Z8_TIMERCTL_DIV64    _HX(30)
#define Z8_TIMERCTL_DIV128   _HX(38)
#define Z8_TIMERCTL_ONESHOT  _HX(00) /* Bits 0-2: Timer mode */
#define Z8_TIMERCTL_CONT     _HX(01)
#define Z8_TIMERCTL_COUNTER  _HX(02)
#define Z8_TIMERCTL_PWM      _HX(03)
#define Z8_TIMERCTL_CAPTURE  _HX(04)
#define Z8_TIMERCTL_COMPARE  _HX(05)
#define Z8_TIMERCTL_GATED    _HX(06)
#define Z8_TIMERCTL_CAPCMP   _HX(07)

/* Register access macros ***********************************************************
 *
 * The register access mechanism provided in ez8.h differs from the useful in other
 * NuttX architectures.  The following NuttX common macros will at least make the
 * access compatible at the source level (however, strict type check is lost).
 */

#ifndef __ASSEMBLY__
# define getreg8(a)           (a)
# define putreg8(v,a)         (a) = (v)
# define getreg16(a)          (a)
# define putreg16(v,a)        (a) = (v)
# define getreg32(a)          (a)
# define putreg32(v,a)        (a) = (v)
#endif /* __ASSEMBLY__ */

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

#endif  /* __Z8_CHIP_H */

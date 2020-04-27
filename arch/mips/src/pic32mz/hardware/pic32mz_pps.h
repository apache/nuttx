/********************************************************************************************
 * arch/mips/src/pic32mz/hardware/pic32mz_pps.h
 *
 *   Copyright (C) 2015, 2018 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

#ifndef __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_PPS_H
#define __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_PPS_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* PPS Register Offsets/Addresses ***********************************************************/

/* Depends on the peripherals supported by the particular device */

#if defined(CONFIG_ARCH_CHIP_PIC32MZEC)
#  include "hardware/pic32mzec_pps.h"
#elif defined(CONFIG_ARCH_CHIP_PIC32MZEF)
#  include "hardware/pic32mzef_pps.h"
#else
#  error Unknown PIC32MZ family
#endif

/* PPS Register Bit Field Definitions *************************************(*****************/
/* All registers contain a single 4 bit field (bits 0-3) holding the peripheral pin
 * selection.
 */

#define PPS_MASK 0x0000000f

/* Pin Selection Helper Macros **************************************************************/
/* The encoding of the input pin selection is simple.  Since we know the devices, we also
 * can infer the register address so we need only the value for the register which is
 * exactly what is provided by macro definitions.
 *
 * The encoding of the output pin selection is a little more complex.  Knowing the device
 * does not provide sufficient information.  The output pin definitions include both the
 * register value and the register address and the following helper macros can be used
 * extract one or the other.
 *
 * NOTE: These odd macro forms are used to work around a pre-processor issue.  The argument
 * to PPS_OUTPUT_REGADDR is defined to have the form nn,xxxx but the preprocessor would
 * claim that only one parameter is passed.  The following version takes only one parameter
 * and keeps the pre-processor happy.
 */

#define __PPS_OUTPUT_REGADDR(a,b) ((uintptr_t)(b))
#define PPS_OUTPUT_REGADDR(a)  __PPS_OUTPUT_REGADDR(a)

#define __PPS_OUTPUT_REGVAL(a,b)  ((uint32_t)(a))
#define PPS_OUTPUT_REGVAL(a)  __PPS_OUTPUT_REGVAL(a)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

#ifndef __ASSEMBLY__

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_PPS_H */

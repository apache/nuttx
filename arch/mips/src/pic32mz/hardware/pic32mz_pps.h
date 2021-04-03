/****************************************************************************
 * arch/mips/src/pic32mz/hardware/pic32mz_pps.h
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
 ****************************************************************************/

#ifndef __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_PPS_H
#define __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_PPS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* PPS Register Offsets/Addresses *******************************************/

/* Depends on the peripherals supported by the particular device */

#if defined(CONFIG_ARCH_CHIP_PIC32MZEC)
#  include "hardware/pic32mzec_pps.h"
#elif defined(CONFIG_ARCH_CHIP_PIC32MZEF)
#  include "hardware/pic32mzef_pps.h"
#else
#  error Unknown PIC32MZ family
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PPS Register Bit Field Definitions ***************************************/

/* All registers contain a single 4 bit field (bits 0-3) holding the
 * peripheral pin selection.
 */

#define PPS_MASK 0x0000000f

/* Pin Selection Helper Macros **********************************************/

/* The encoding of the input pin selection is simple.
 * Since we know the devices, we also can infer the register address so we
 * need only the value for the register which is exactly what is provided
 * by macro definitions.
 *
 * The encoding of the output pin selection is a little more complex.
 * Knowing the device does not provide sufficient information.  The output
 * pin definitions include both the register value and the register address
 * and the following helper macros can be used extract one or the other.
 *
 * NOTE: These odd macro forms are used to work around a pre-processor issue.
 * The argument to PPS_OUTPUT_REGADDR is defined to have the form nn,xxxx
 * but the preprocessor would claim that only one parameter is passed.
 *  The following version takes only one parameter and keeps the
 * pre-processor happy.
 */

#define __PPS_OUTPUT_REGADDR(a,b) ((uintptr_t)(b))
#define PPS_OUTPUT_REGADDR(a)  __PPS_OUTPUT_REGADDR(a)

#define __PPS_OUTPUT_REGVAL(a,b)  ((uint32_t)(a))
#define PPS_OUTPUT_REGVAL(a)  __PPS_OUTPUT_REGVAL(a)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_PPS_H */

/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx_reset.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_RESET_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_RESET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "pic32mx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define PIC32MX_RESET_RCON_OFFSET      0x0000 /* Reset control register */
#define PIC32MX_RESET_RCONCLR_OFFSET   0x0004 /* RCON clear register */
#define PIC32MX_RESET_RCONSET_OFFSET   0x0008 /* RCON set register */
#define PIC32MX_RESET_RCONINV_OFFSET   0x000c /* RCON invert register */
#define PIC32MX_RESET_RSWRST_OFFSET    0x0010 /* Software reset register */
#define PIC32MX_RESET_RSWRSTCLR_OFFSET 0x0014 /* RSWRST clear register */
#define PIC32MX_RESET_RSWRSTSET_OFFSET 0x0018 /* RSWRST set register */
#define PIC32MX_RESET_RSWRSTINV_OFFSET 0x001c /* RSWRST invert register */

/* Register Addresses *******************************************************/

#define PIC32MX_RESET_RCON             (PIC32MX_RESET_K1BASE+PIC32MX_RCON_OFFSET)
#define PIC32MX_RESET_RCONCLR          (PIC32MX_RESET_K1BASE+PIC32MX_RCONCLR_OFFSET)
#define PIC32MX_RESET_RCONSET          (PIC32MX_RESET_K1BASE+PIC32MX_RCONSET_OFFSET)
#define PIC32MX_RESET_RCONINV          (PIC32MX_RESET_K1BASE+PIC32MX_RCONINV_OFFSET)
#define PIC32MX_RESET_RSWRST           (PIC32MX_RESET_K1BASE+PIC32MX_RSWRST_OFFSET)
#define PIC32MX_RESET_RSWRSTCLR        (PIC32MX_RESET_K1BASE+PIC32MX_RSWRSTCLR_OFFSET)
#define PIC32MX_RESET_RSWRSTSET        (PIC32MX_RESET_K1BASE+PIC32MX_RSWRSTSET_OFFSET)
#define PIC32MX_RESET_RSWRSTINV        (PIC32MX_RESET_K1BASE+PIC32MX_RSWRSTINV_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Reset control register */

#define RESET_RCON_POR                 (1 << 0)  /* Bit 0: Power on reset */
#define RESET_RCON_BOR                 (1 << 1)  /* Bit 1: Brown out reset */
#define RESET_RCON_IDLE                (1 << 2)  /* Bit 2: Wake from idle */
#define RESET_RCON_SLEEP               (1 << 3)  /* Bit 3: Wake from sleep */
#define RESET_RCON_WDTO                (1 << 4)  /* Bit 4: Watchdog timer time-out */
#define RESET_RCON_SWR                 (1 << 6)  /* Bit 6: Software reset */
#define RESET_RCON_EXTR                (1 << 7)  /* Bit 7: External reset pin */
#define RESET_RCON_VREGS               (1 << 8)  /* Bit 8: Voltage regulator standby enable */
#define RESET_RCON_CMR                 (1 << 9)  /* Bit 9: Configuration mismatch reset */

/* Software reset register */

#define RESET_RSWRST_TRIGGER           (1 << 0)  /* Bit 0: Software reset trigger */

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
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_RESET_H */

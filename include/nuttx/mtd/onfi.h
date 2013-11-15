/****************************************************************************
 * include/nuttx/mtd/onfi.h
 *
 * ONFI Support.  The Open NAND Flash Interface (ONFI) is an industry
 * Workgroup made up of more than 100 companies that build, design-in, or
 * enable NAND Flash memory. This file provides definitions for standardized
 * ONFI NAND interfaces.
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This ONFI logic was based largely on Atmel sample code with modifications
 * for better integration with NuttX.  The Atmel sample code has a BSD
 * compatibile license that requires this copyright notice:
 *
 *   Copyright (c) 2010, Atmel Corporation
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
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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

#ifndef __INCLUDE_NUTTX_MTD_ONFI_H
#define __INCLUDE_NUTTX_MTD_ONFI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Opaque handler used to interact with the ONFI module */

typedef FAR void *ONFI_HANDLE;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

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

/****************************************************************************
 * Name: onfi_compatible
 *
 * Description:
 *   This function read an the ONFI signature at address of 20h to detect
 *   if the device is ONFI compatiable.
 *
 * Input Parameters:
 *   cmdaddr  - NAND command address base
 *   addraddr - NAND address address base
 *   dataaddr - NAND data address
 *
 * Returned Value:
 *   True if ONFI compatible
 *
 ****************************************************************************/

bool onfi_compatible(uintptr_t cmdaddr, uintptr_t addraddr,
                     uintptr_t dataaddr);

/****************************************************************************
 * Name: onfi_create
 *
 * Description:
 *   If the addresses refere to a compatible ONFI device, then create the
 *   ONFI handle that can be used to interact with the device.
 *
 * Input Parameters:
 *   cmdaddr  - NAND command address base
 *   addraddr - NAND address address base
 *   dataaddr - NAND data address
 *
 * Returned Value:
 *   On success, a non-NULL ONFI handle is returned.  This handle must be
 *   freed by calling onfi_destroy with it is no longer needed.
 *   NULL is returned on any failure.  Failures include such things as
 *   memory allocation failures, ONFI incompatibility, timeouts, etc.
 *
 ****************************************************************************/

ONFI_HANDLE *onfi_create(uintptr_t cmdaddr, uintptr_t addraddr,
                         uintptr_t dataaddr);

/****************************************************************************
 * Name: onfi_destroy
 *
 * Description:
 *   Free resources allocated on onfi_create() when the ONFI handle was
 *   created.  Upon return, the ONFI handle is no longer valid and should not
 *   be used further.
 *
 * Input Parameters:
 *   handle - An ONFI handle previously created by onfi_create().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void onfi_destroy(ONFI_HANDLE handle);

/****************************************************************************
 * Name: onfi_embeddedecc
 *
 * Description:
 *   Enable or disable the NAND's embedded ECC controller.
 *
 * Input Parameters:
 *   handle - An ONFI handle previously created by onfi_create().
 *   enable - True: enabled the embedded ECC function; False: disable it
 *
 * Returned Value:
 *   True  - Internal ECC enabled or disabled successfully
 *   False - Internal ECC not supported.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_NAND_EMBEDDEDECC
bool onfi_embeddedecc(ONFI_HANDLE handle, bool enable);
#endif

/****************************************************************************
 * Name: onfi_ebidetect
 *
 * Description:
 *   Detect Nand connection on EBI
 *
 * Input Parameters:
 *   cmdaddr  - NAND command address base
 *   addraddr - NAND address address base
 *   dataaddr - NAND data address
 *
 * Returned Value:
 *   True if the chip is detected; false otherwise.
 *
 ****************************************************************************/

bool onfi_ebidetect(uintptr_t cmdaddr, uintptr_t addraddr, uintptr_t
                    dataaddr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_MTD_ONFI_H */

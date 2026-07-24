/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_pagefault.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_PAGEFAULT_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_PAGEFAULT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_pagefault_dispatch
 *
 * Description:
 *   Service a precise PMS permission fault (EXCCAUSE Load/Store/InstrFetch
 *   Prohibited).  This is the recoverable-fault entry point invoked from the
 *   Xtensa user exception handler (xtensa_user()).
 *
 *   The faulting data address is read from the register save area
 *   (regs[REG_EXCVADDR]).  If the fault is serviced, the handler leaves the
 *   saved PC (regs[REG_PC] == EPC1) unchanged so that, upon return, the RFE
 *   in the exception vector re-executes the faulting instruction.
 *
 * Input Parameters:
 *   exccause - The EXCCAUSE value (20/28/29).
 *   regs     - Pointer to the register save area.
 *
 * Returned Value:
 *   OK if the fault was serviced and the instruction should be retried via
 *   RFE; a negated errno value if the fault is not recoverable (the caller
 *   then panics / terminates the faulting task).
 *
 ****************************************************************************/

int esp32s3_pagefault_dispatch(int exccause, uint32_t *regs);

#ifdef CONFIG_ESP32S3_PAGEFAULT_ABORT
/****************************************************************************
 * Name: esp32s3_pagefault_abort
 *
 * Description:
 *   Terminate just the faulting unprivileged (WORLD1) task via a fatal
 *   SIGSEGV instead of panicking the whole system.  Returns the register
 *   frame to resume (the signal trampoline for the faulting task).
 *
 ****************************************************************************/

uint32_t *esp32s3_pagefault_abort(int exccause, uint32_t *regs);
#endif

#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_PAGEFAULT_H */

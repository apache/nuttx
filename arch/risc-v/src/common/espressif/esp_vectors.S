/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_vectors.S
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

#include "chip.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  .global  _vector_table

/****************************************************************************
 * Section: .exception_vectors.text
 ****************************************************************************/

  .section .exception_vectors.text

/****************************************************************************
 * Name: _vector_table
 ****************************************************************************/

  .balign   0x100
  .type     _vector_table, @function

_vector_table:
  .option push
  .option norvc

  j _panic_handler            /* 0: Exception entry */
  j exception_common        /* 1: Free interrupt number */
  j exception_common        /* 2: Free interrupt number */
  j exception_common        /* 3: Free interrupt number */
  j exception_common        /* 4: Free interrupt number */
  j exception_common        /* 5: Free interrupt number */
  j exception_common        /* 6: Free interrupt number */
  j exception_common        /* 7: Free interrupt number */
  j exception_common        /* 8: Free interrupt number */
  j exception_common        /* 9: Free interrupt number */
  j exception_common        /* 10: Free interrupt number */
  j exception_common        /* 11: Free interrupt number */
  j exception_common        /* 12: Free interrupt number */
  j exception_common        /* 13: Free interrupt number */
  j exception_common        /* 14: Free interrupt number */
  j exception_common        /* 15: Free interrupt number */
  j exception_common        /* 16: Free interrupt number */
  j exception_common        /* 17: Free interrupt number */
  j exception_common        /* 18: Free interrupt number */
  j exception_common        /* 19: Free interrupt number */
  j exception_common        /* 20: Free interrupt number */
  j exception_common        /* 21: Free interrupt number */
  j exception_common        /* 22: Free interrupt number */
  j exception_common        /* 23: Free interrupt number */
  j _panic_handler            /* 24: ETS_INT_WDT_INUM panic-interrupt (soc-level panic) */
  j _panic_handler            /* 25: ETS_CACHEERR_INUM panic-interrupt (soc-level panic) */
  j _panic_handler               /* 26: ETS_MEMPROT_ERR_INUM handler (soc-level panic) */
  j _panic_handler                /* 27: ETS_ASSIST_DEBUG_INUM handler (soc-level panic) */
  j exception_common        /* 28: Free interrupt number */
  j exception_common        /* 29: Free interrupt number */
  j exception_common        /* 30: Free interrupt number */
  j exception_common        /* 31: Free interrupt number */

/****************************************************************************
 * Name: exception_common
 *
 * Description:
 *   Handles interrupts. If kernel is in S-mode, handles delegated interrupts
 *   in S-mode interrupt handler.
 *
 ****************************************************************************/

  .global _panic_handler

_panic_handler:
  j exception_common /* Dispatch the system call */

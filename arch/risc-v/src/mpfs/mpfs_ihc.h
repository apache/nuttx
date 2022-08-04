/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_ihc.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_MPFS_IHC_H
#define __ARCH_RISCV_SRC_MPFS_MPFS_IHC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: mpfs_ihc_init
 *
 * Description:
 *   This initializes the Inter-Hart Communication (IHC) module.  Rptun is
 *   used to simplify the integration of rpmsg and virtio.  This function
 *   installs the proper interrupt handlers, installs a thread, and performs
 *   all the required initialization tasks.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success, a nagated errno on error
 *
 ****************************************************************************/

int mpfs_ihc_init(void);

/****************************************************************************
 * Name: mpfs_ihc_sbi_ecall_handler
 *
 * Description:
 *   This is sbi_platform_operations / vendor_ext_provider ecall handler.
 *   Related Linux ecalls end up here.
 *
 * Input Parameters:
 *   funcid          - SBI_EXT_IHC_CTX_INIT, SBI_EXT_IHC_SEND or
 *                     SBI_EXT_IHC_RECEIVE.  Others are invalid.
 *   remote_channel  - The remote we're communicating with
 *   message_ptr     - Local storage for data exchange
 *
 * Returned Value:
 *   OK on success, a negated error code otherwise
 *
 ****************************************************************************/

int mpfs_ihc_sbi_ecall_handler(unsigned long funcid, uint32_t remote_channel,
                               uint32_t *message_ptr);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_MPFS_MPFS_IHC_H */

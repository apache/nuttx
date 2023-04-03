/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_dsn.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_MPFS_DSN_H
#define __ARCH_RISCV_SRC_MPFS_MPFS_DSN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>

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
 * Pre-Processor Declarations
 ****************************************************************************/

/* This is the length of the serial number */

#define MPFS_DSN_LENGTH    16

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_read_dsn
 *
 * Description:
 *   Read n bytes of the device serial number. Full serial number is 16 bytes
 *
 * Parameters:
 *   dsn - A pointer to the destination buffer
 *   len - Number of bytes to read
 *
 * Returned Value:
 *   Number of bytes read or negated errno
 *
 ****************************************************************************/

int mpfs_read_dsn(uint8_t *dsn, size_t len);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_MPFS_MPFS_DSN_H */

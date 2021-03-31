/****************************************************************************
 * arch/arm/include/cxd56xx/backuplog.h
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

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_BACKUPLOG_H
#define __ARCH_ARM_INCLUDE_CXD56XX_BACKUPLOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: up_backuplog_initialize
 *
 * Description:
 *   Initialize the log header where the address and size of each log area
 *   are described. If the log header has been already configured as a wakeup
 *   from sleeping or reboot case, then do nothing and return OK.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int up_backuplog_initialize(void);

/****************************************************************************
 * Name: up_backuplog_alloc
 *
 * Description:
 *   Allocate the log memory region
 *
 * Input Parameters:
 *   name - The log region name
 *   size - The size to allocate
 *
 * Returned Value:
 *   The allocated address on success; NULL value on failure.
 *
 ****************************************************************************/

void *up_backuplog_alloc(const char *name, size_t size);

/****************************************************************************
 * Name: up_backuplog_free
 *
 * Description:
 *   De-allocate the log memory region
 *
 * Input Parameters:
 *   name - The log region name
 *
 ****************************************************************************/

void up_backuplog_free(const char *name);

/****************************************************************************
 * Name: up_backuplog_region
 *
 * Description:
 *   Get the address and size of the specified log region name
 *
 * Input Parameters:
 *   name - The log region name
 *   addr - The returned address
 *   size - The returned size
 *
 * Returned Value:
 *   The index of log entry on success; A negated errno value on failure.
 *
 ****************************************************************************/

int up_backuplog_region(const char *name, void **addr, size_t *size);

/****************************************************************************
 * Name: up_backuplog_entry
 *
 * Description:
 *   Get the entry name, address and size
 *
 * Input Parameters:
 *   name - The returned entry name
 *   addr - The returned address
 *   size - The returned size
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int up_backuplog_entry(char *name, void **addr, size_t *size);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_INCLUDE_CXD56XX_BACKUPLOG_H */

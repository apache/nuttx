/****************************************************************************
 * arch/arm/include/cxd56xx/backuplog.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

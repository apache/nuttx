/****************************************************************************
 * include/nuttx/elf.h
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

#ifndef __INCLUDE_NUTTX_ELF_H
#define __INCLUDE_NUTTX_ELF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <elf.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Name: up_checkarch
 *
 * Description:
 *   Given the ELF header in 'hdr', verify that the module is appropriate
 *   for the current, configured architecture.  Every architecture that uses
 *   the module loader must provide this function.
 *
 * Input Parameters:
 *   hdr - The ELF header read from the module file.
 *
 * Returned Value:
 *   True if the architecture supports this module file.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_ARCH_ELF
bool up_checkarch(FAR const Elf_Ehdr *hdr);
#endif

/****************************************************************************
 * Name: up_relocate and up_relocateadd
 *
 * Description:
 *   Perform on architecture-specific ELF relocation.  Every architecture
 *   that uses the module loader must provide this function.
 *
 * Input Parameters:
 *   rel - The relocation type
 *   sym - The ELF symbol structure containing the fully resolved value.
 *         There are a few relocation types for a few architectures that do
 *         not require symbol information.  For those, this value will be
 *         NULL.  Implementations of these functions must be able to handle
 *         that case.
 *   addr - The address that requires the relocation.
 *
 * Returned Value:
 *   Zero (OK) if the relocation was successful.  Otherwise, a negated errno
 *   value indicating the cause of the relocation failure.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_ARCH_ELF
int up_relocate(FAR const Elf_Rel *rel, FAR const Elf_Sym *sym,
                uintptr_t addr);
int up_relocateadd(FAR const Elf_Rela *rel,
                   FAR const Elf_Sym *sym, uintptr_t addr);
#endif

/****************************************************************************
 * Name: up_init_exidx
 *
 * Description:
 *   Initialize the exception index section.
 *
 * Input Parameters:
 *   address - The exception index section address.
 *   size    - The exception index section size.
 *
 * Returned Value:
 *   Zero (OK) if the initialization was successful. Otherwise, a negated
 *   errno value indicating the cause of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_CXX_EXCEPTION
int up_init_exidx(Elf_Addr address, Elf_Word size);
#endif

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_ELF_H */

/****************************************************************************
 * tools/nxflat/nxflat_thunk.h
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

#ifndef __TOOLS_NXFLAT_NXFLAT_THUNK_H
#define __TOOLS_NXFLAT_NXFLAT_THUNK_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The register a module reaches its own data through, named here once so
 * the thunk stubs and arch/arm cannot drift apart.  It must agree with
 * PIC_REG in arch/arm/include/arch.h and with -mpic-register in
 * arch/arm/src/common/Toolchain.defs: a module whose data accesses use one
 * register and whose import thunks add another will load, and then branch
 * to a wild address on its first call out.
 */

#define NXFLAT_PIC_REG "r9"

/* Every module declares the module ABI it was built for by importing this
 * name, and the loader refuses a module that does not.  That is the only
 * version channel available: the NXFLAT header has no version field, and
 * h_magic is written by ldnxflat, which is GPL and stays out of the
 * repository, so it can never be changed in step with the loader.  The
 * import table can, because mknxflat emits it and nxflat_bindimports()
 * reads it, and ldnxflat passes it through untouched.
 *
 * Must match NXFLAT_ABI_SYMBOL in include/nxflat.h.
 */

#define NXFLAT_ABI_SYMBOL "__nxflat_abi_v2"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The set of format strings that make up one architecture's thunk file.
 *
 * The upstream tool selected these at compile time through an "arch"
 * symlink pointing at either arm/ or thumb2/.  A symlink cannot be carried
 * in the repository, and one host binary has to serve boards of both
 * flavours -- lpc31xx is ARM while lpc17xx, tiva, stm32f1 and rp23xx are
 * Thumb-2 -- so the choice moves to a runtime "-a" option instead.  The
 * .def files themselves are unmodified, so the emitted text is unchanged.
 */

struct nxflat_thunk_s
{
  const char *file_prologue;
  const char *import_prologue;
  const char *import_name_strtab_prologue;
  const char *import_name_strtab_format;
  const char *dynimport_decl_prologue;
  const char *dynimport_decl_format;
  const char *dynimport_array_prologue;
  const char *dynimport_array_format;
  const char *dynimport_array_epilogue;
  const char *dyncall_decl_prologue;
  const char *dyncall_format;
  const char *nonreturning_dyncall_format;
  const char *file_epilogue;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const struct nxflat_thunk_s g_thunk_arm;
extern const struct nxflat_thunk_s g_thunk_thumb2;

#endif /* __TOOLS_NXFLAT_NXFLAT_THUNK_H */

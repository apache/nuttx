/****************************************************************************
 * tools/nxflat/thunk_thumb2.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2009, 2018 Gregory Nutt. All rights reserved.
 * SPDX-FileCopyrightText: 2002, 2006 Cadenux, LLC. All rights reserved.
 * SPDX-FileContributor: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES.  See the .def file
 * included below, which is carried unmodified from the upstream NuttX
 * buildroot NXFLAT toolchain.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "nxflat_thunk.h"

/* The format strings are file-scope statics inside the .def, so each
 * architecture gets its own translation unit and the two sets cannot
 * collide.  The .def is byte-for-byte the upstream file.
 */

#include "dyncall_skeleton_thumb2.def"

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct nxflat_thunk_s g_thunk_thumb2 =
{
  file_prologue,
  import_prologue,
  import_name_strtab_prologue,
  import_name_strtab_format,
  dynimport_decl_prologue,
  dynimport_decl_format,
  dynimport_array_prologue,
  dynimport_array_format,
  dynimport_array_epilogue,
  dyncall_decl_prologue,
  dyncall_format,
  nonreturning_dyncall_format,
  file_epilogue
};

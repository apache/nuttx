/****************************************************************************
 * tools/nxflat/nxflat_thunk.h
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

#ifndef __TOOLS_NXFLAT_NXFLAT_THUNK_H
#define __TOOLS_NXFLAT_NXFLAT_THUNK_H

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

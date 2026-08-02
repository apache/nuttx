/****************************************************************************
 * tools/nxflat/thunk_arm.c
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

#include "nxflat_thunk.h"

/* The format strings have file scope inside the .def, so each
 * architecture gets its own translation unit and the two sets cannot
 * collide.  The .def is the upstream file, less one comment typo.
 */

#include "dyncall_skeleton_arm.def"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* None: this translation unit exists only to give one architecture's
 * format strings a scope of their own.
 */

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct nxflat_thunk_s g_thunk_arm =
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

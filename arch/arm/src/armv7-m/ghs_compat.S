/****************************************************************************
 * arch/arm/src/armv7-m/ghs_compat.S
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

#ifdef __ghs__
    .macro  .cfi_sections section
    .endm

    .macro  .cfi_startproc
    .endm

    .macro  .cfi_def_cfa val1 val2
    .endm

    .macro  .cfi_def_cfa_offset val
    .endm

    .macro  .cfi_offset val1 val2
    .endm

    .macro  .cfi_restore val
    .endm

    .macro  .cfi_restore_state
    .endm

    .macro  .cfi_endproc
    .endm

    .macro  .cfi_remember_state
    .endm
#endif

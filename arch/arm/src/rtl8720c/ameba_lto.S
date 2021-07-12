/****************************************************************************
 * arch/arm/src/rtl8720c/ameba_lto.S
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
 * Public Symbols
 ****************************************************************************/

  .syntax	unified
  .arch armv8-m.main
  .thumb
  .file	"ameba_lto.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

  .text
  .thumb_func
  .globl ameba_lto
  .globl __aeabi_d2iz
  .globl __aeabi_d2lz
  .globl __aeabi_d2ulz
  .globl __aeabi_dcmpeq
  .globl __aeabi_dcmpge
  .globl __aeabi_dcmplt
  .globl __aeabi_ddiv
  .globl __aeabi_fadd
  .globl __aeabi_fcmpge
  .globl __aeabi_fcmpgt
  .globl __aeabi_fdiv
  .globl __aeabi_fmul
  .globl __aeabi_i2f
  .globl __aeabi_idivmod
  .globl __aeabi_ldivmod
  .globl __aeabi_llsl
  .globl __aeabi_llsr
  .globl __aeabi_lmul
  .globl __aeabi_uidiv
  .globl __aeabi_uldivmod
  .globl __bswapsi2
  .globl __clzsi2
  .globl __ctzsi2
  .globl __gnu_thumb1_case_shi
  .globl __gnu_thumb1_case_si
  .globl __gnu_thumb1_case_sqi
  .globl __gnu_thumb1_case_uhi
  .globl __gnu_thumb1_case_uqi
  .globl __gnu_thumb1_case_uqi
  .globl __popcountsi2
  .type	ameba_lto, function
ameba_lto:
  .size	ameba_lto, .-ameba_lto
  .end

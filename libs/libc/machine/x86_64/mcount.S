/****************************************************************************
 * libs/libc/machine/x86_64/gnu/mcount.S
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
 * Public Functions
 ****************************************************************************/

  .globl  mcount
  .type   mcount, @function

mcount:

   /* Allocate space for 7 registers */

   subq   $56,    %rsp
   movq   %rax,   (%rsp)
   movq   %rcx,   8(%rsp)
   movq   %rdx,   16(%rsp)
   movq   %rsi,   24(%rsp)
   movq   %rdi,   32(%rsp)
   movq   %r8,    40(%rsp)
   movq   %r9,    48(%rsp)

   movq   56(%rsp),%rsi

   /* Get frompc via the fp */

   movq   8(%rbp), %rdi

   /* Jump to mcount_internal */

   call   mcount_internal

   /* Pop the saved registers.  Please note that `mcount' has no
      return value.
    */

   movq   48(%rsp),%r9
   movq   40(%rsp),%r8
   movq   32(%rsp),%rdi
   movq   24(%rsp),%rsi
   movq   16(%rsp),%rdx
   movq   8(%rsp), %rcx
   movq   (%rsp),  %rax
   addq   $56,     %rsp
   ret

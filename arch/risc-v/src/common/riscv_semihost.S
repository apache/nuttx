/****************************************************************************
 * arch/risc-v/src/common/riscv_semihost.S
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
 * Name: smh_call
 *
 * Description:
 *   Semihosting call with call number and one parameter
 *
 ****************************************************************************/

  .option norvc
  .text
  .balign 16
  .global smh_call
  .type smh_call @function

smh_call:

  slli zero, zero, 0x1f
  ebreak
  srai zero, zero, 0x7
  ret

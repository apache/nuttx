/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_vectors.S
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

  .section .text.vec
  .global  __reset_vec
  .global  __trap_vec
  .global  __start

/****************************************************************************
 * Name: __reset_vec
 * 
 * Description:
 *   Also __start symbol is defined to be on start of image. This is expected
 *   by some of Microchip tools, which can be used to e.g. flash the image
 *
 ****************************************************************************/

__start:
__reset_vec:
  j    __start_mpfs

/****************************************************************************
 * Name: exception_common
 *
 * Description:
 *   All exceptions and interrupts will be handled from here.
 *
 ****************************************************************************/

__trap_vec:
  j    exception_common
  nop

/****************************************************************************
 * boards/risc-v/k210/maix-bit/scripts/memory.ld
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

/* Reg   Access        Start addr    End addr     Size
 * MEM0  CPU w/  cache  0x80000000 - 0x803fffff : 4MB
 * MEM1  CPU w/  cache  0x80400000 - 0x805fffff : 2MB
 * MEM0  CPU w/o cache  0x40000000 - 0x403fffff : 4MB
 * MEM1  CPU w/o cache  0x40400000 - 0x405fffff : 4MB
 */

MEMORY
{
  kflash (rx)  : ORIGIN = 0x80000000, LENGTH = 1024K  /* w/ cache */
  uflash (rx)  : ORIGIN = 0x80100000, LENGTH = 1024K  /* w/ cache */
  xflash (rx)  : ORIGIN = 0x80200000, LENGTH = 2048K  /* w/ cache */

  ksram (rwx)  : ORIGIN = 0x80400000, LENGTH = 512K   /* w/ cache */
  usram (rwx)  : ORIGIN = 0x80480000, LENGTH = 512K   /* w/ cache */
  xsram (rwx)  : ORIGIN = 0x80500000, LENGTH = 1024K  /* w/ cache */
}

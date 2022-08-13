/****************************************************************************
 * boards/risc-v/mpfs/m100pfsevp/scripts/memory.ld
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
 * QEMU  CPU w/  cache  0x00000000 - 0x003fffff : 4MB
 * QEMU  CPU w/o cache  0x1f000000 - 0x1f01ffff : 128KB
 */

MEMORY
{
  kflash (rx)  : ORIGIN = 0x80000000, LENGTH = 256K /* w/ cache */
  uflash (rx)  : ORIGIN = 0x80040000, LENGTH = 256K /* w/ cache */
  xflash (rx)  : ORIGIN = 0x80080000, LENGTH = 256K   /* w/o cache */

  ksram (rwx)  : ORIGIN = 0x800C0000, LENGTH = 256K /* w/ cache */
  usram (rwx)  : ORIGIN = 0x80100000, LENGTH = 256K /* w/ cache */
  xsram (rwx)  : ORIGIN = 0x80140000, LENGTH = 256K   /* w/o cache */
}

/****************************************************************************
 * scripts/memory.ld
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

/*
 *
 *   0x00400000 - 0x007fffff  4194304  Program Flash (last 64K sBAF)
 *   0x10000000 - 0x1003ffff   262144  Data Flash (last 32K HSE_NVM)
 *   0x20400000 - 0x20408000    32768  Standby RAM_0 (32K)
 *   0x20400000 - 0x20427fff   163840  SRAM_0
 *   0x20428000 - 0x2044ffff   163840  SRAM_1
 *
 *   Last  48 KB of SRAM_1 reserved by HSE Firmware
 *   Last 128 KB of CODE_FLASH_3 reserved by HSE Firmware
 *   Last 128 KB of DATA_FLASH reserved by HSE Firmware (not supported in this linker file)
 *
 *   Note Standby RAM and SRAM overlaps in NuttX since we dont use the Standby functionality
 *
 */

MEMORY
{
  /* 4096KB FLASH*/

  BOOT_HEADER (R)   : ORIGIN = 0x00400000, LENGTH = 0x00001000  /* 0x00400000 - 0x00400fff */
  flash       (rx)  : ORIGIN = 0x00401000, LENGTH = 0x003cffff
  kflash      (rx)  : ORIGIN = 0x00401000, LENGTH = 1024K
  uflash      (rx)  : ORIGIN = 0x00401000+1024K, LENGTH = 0x003cffff-1024K

  /* ITCM RAM */

  itcm        (rwx) : ORIGIN = 0x00000000, LENGTH = 64K

  /* DTCM SRAM */

  dtcm        (rwx) : ORIGIN = 0x20000000, LENGTH = 128K

  /* SRAM */
  sram0_stdby (rwx) : ORIGIN = 0x20400000, LENGTH = 32K
  sram  (rwx) : ORIGIN = 0x20400000, LENGTH = 272K
  ksram (rwx) : ORIGIN = 0x20400000, LENGTH = 128K
  usram (rwx) : ORIGIN = 0x20400000+128K, LENGTH = 272K-128K
}


  CM7_0_START_ADDRESS    = ORIGIN(flash);
  SRAM_BASE_ADDR         = ORIGIN(sram);
  SRAM_END_ADDR          = ORIGIN(sram) + LENGTH(sram);
  SRAM_STDBY_BASE_ADDR   = ORIGIN(sram0_stdby);
  SRAM_STDBY_END_ADDR    = ORIGIN(sram0_stdby) + LENGTH(sram0_stdby);
  SRAM_INIT_END_ADDR     = ORIGIN(sram) + 320K;
  ITCM_BASE_ADDR         = ORIGIN(itcm);
  ITCM_END_ADDR          = ORIGIN(itcm) + LENGTH(itcm);
  DTCM_BASE_ADDR         = ORIGIN(dtcm);
  DTCM_END_ADDR          = ORIGIN(dtcm) + LENGTH(dtcm);
  FLASH_BASE_ADDR        = ORIGIN(BOOT_HEADER);
  FLASH_END_ADDR         = ORIGIN(flash) + LENGTH(flash);

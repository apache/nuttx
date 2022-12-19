/****************************************************************************
 * scripts/memory.ld
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

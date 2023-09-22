/****************************************************************************
 * arch/arm/src/mx8mp/mx8mp_mpuinit.c
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

#include <nuttx/config.h>

#include <assert.h>
#include <sys/param.h>

#include <nuttx/userspace.h>

#include "mpu.h"
#include "mx8mp_mpuinit.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx8mp_mpu_initialize
 *
 * Description:
 *   Configure the MPU to permit access to imx8mp SoC complex.
 *
 ****************************************************************************/

void mx8mp_mpu_initialize(void)
{
  /* Show MPU information */

  mpu_showtype();

  /* Reset MPU if enabled */

  mpu_reset();

  /* Region 0 [0x0000_0000 - 0x3FFF_FFFF]
   * Device type, not executable, not shareable, non-cacheable.
   */

  mpu_configure_region(0x00000000, 1 * 1024 * 1024 * 1024,
                      MPU_RASR_TEX_DEV  |  /* Device
                                            * Not Cacheable  */
                      MPU_RASR_B        |  /* Bufferable
                                            * Not Shareable  */
                      MPU_RASR_AP_RWRW  |  /* P:RW   U:RW    */
                      MPU_RASR_XN);        /* Not executable */

  /* Region 1 TCML[0x0000_0000 - 0x0001_FFFF]
   * Normal type, not shareable, non-cacheable
   */

  mpu_configure_region(0x00000000, 128 * 1024,
                      MPU_RASR_TEX_NOR  |  /* Normal
                                            * Not Cacheable  */
                      MPU_RASR_B        |  /* Bufferable
                                            * Not Shareable  */
                      MPU_RASR_AP_RWRW);   /* P:RW   U:RW
                                            * Executable */

  /* Region 2 QSPI[0x0800_0000 - 0x0FFF_FFFF]
   * Normal type, not shareable, cacheable
   */

  mpu_configure_region(0x08000000, 128 * 1024 * 1024,
                      MPU_RASR_TEX_NOR  |  /* Normal */
                      MPU_RASR_C        |  /* Cacheable  */
                      MPU_RASR_B        |  /* Bufferable
                                            * Not Shareable  */
                      MPU_RASR_AP_RWRW);   /* P:RW   U:RW
                                            * Executable */

  /* Region 3 TCMU[0x2000_0000 - 0x2001_FFFF]
   * Normal type, not shareable, non-cacheable
   */

  mpu_configure_region(0x20000000, 128 * 1024,
                      MPU_RASR_TEX_NOR  |  /* Normal
                                            * Not Cacheable  */
                      MPU_RASR_B        |  /* Bufferable
                                            * Not Shareable  */
                      MPU_RASR_AP_RWRW);   /* P:RW   U:RW
                                            * Executable */

  /* Region 4 DDR[0x4000_0000 - 0x7FFF_FFFF]
   * Normal type, not shareable, cacheable
   */

  mpu_configure_region(0x40000000, 1 * 1024 * 1024 * 1024,
                      MPU_RASR_TEX_NOR  |  /* Normal */
                      MPU_RASR_C        |  /* Cacheable  */
                      MPU_RASR_B        |  /* Bufferable
                                            * Not Shareable  */
                      MPU_RASR_AP_RWRW);   /* P:RW   U:RW
                                            * Executable */
#if 0

#ifdef CONFIG_BUILD_PROTECTED
  uintptr_t datastart = MIN(USERSPACE->us_datastart, USERSPACE->us_bssstart);
  uintptr_t dataend   = MAX(USERSPACE->us_dataend,   USERSPACE->us_bssend);

  DEBUGASSERT(USERSPACE->us_textend >= USERSPACE->us_textstart &&
              dataend >= datastart);

  /* Configure user flash and SRAM space */

  mpu_user_flash(USERSPACE->us_textstart,
                 USERSPACE->us_textend - USERSPACE->us_textstart);
  mpu_user_intsram(datastart, dataend - datastart);
#endif

#endif

  /* Enable MPU and HFNMIENA feature
   * HFNMIENA ensures that M7 core uses MPU configuration when in hard fault,
   * NMI, and FAULTMASK handlers, otherwise all memory regions are accessed
   * without MPU protection, which has high risks of cacheable,
   * especially for AIPS systems.
   */

  mpu_control(true, true, true);
}

/****************************************************************************
 * Name: mx8mp_mpu_uheap
 *
 * Description:
 *  Map the user-heap region.
 *
 *  This logic may need an extension to handle external SDRAM).
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_PROTECTED
void mx8mp_mpu_uheap(uintptr_t start, size_t size)
{
  mpu_user_intsram(start, size);
}
#endif /* CONFIG_BUILD_PROTECTED */

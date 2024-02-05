/****************************************************************************
 * arch/arm/src/imxrt/imxrt117x_mpuinit.c
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
 * Pre-processor Definitions
 ****************************************************************************/

  mpu_reset();

  /* Add default region to deny access to whole address space to workaround
   * speculative prefetch. Refer to Arm errata 1013783-B for more details.
  */

  uint32_t regval;
  uint32_t region;

  region = mpu_allocregion();
  DEBUGASSERT(region == 0);

  /* Select the region */

  putreg32(region, MPU_RNR);

  /* Select the region base address */

  putreg32(region | MPU_RBAR_VALID, MPU_RBAR);

  /* The configure the region */

  regval = MPU_RASR_ENABLE        | /* Enable region  */
           MPU_RASR_SIZE_LOG2(32) | /* entire memory */
           MPU_RASR_TEX_SO        | /* Strongly ordered */
           MPU_RASR_AP_NONO       | /* P:None U:None              */
           MPU_RASR_XN;             /* Execute-never to prevent instruction fetch */
  putreg32(regval, MPU_RASR);

  mpu_configure_region(IMXRT_SEMC0_BASE, 512 * 1024 * 1024,

                                           /* Instruction access Enabled */

                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device                     */
                                           /* Not Cacheable              */
                                           /* Not Bufferable             */
                                           /* Not Shareable              */
                                           /* No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_FLEXSPI2_CIPHER_BASE, 512 * 1024 * 1024,

                                           /* Instruction access Enabled */

                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device                     */
                                           /* Not Cacheable              */
                                           /* Not Bufferable             */
                                           /* Not Shareable              */
                                           /* No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_ITCM_BASE, 1 * 1024 * 1024 * 1024,

                                           /* Instruction access Enabled */

                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device                     */
                                           /* Not Cacheable              */
                                           /* Not Bufferable             */
                                           /* Not Shareable              */
                                           /* No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_ITCM_BASE, 256 * 1024,

                                           /* Instruction access Enabled */

                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_NOR    /* Normal                     */
                                           /* Not Cacheable              */
                                           /* Not Bufferable             */
                                           /* Not Shareable              */
                                           /* No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_DTCM_BASE, 256 * 1024,

                                           /* Instruction access Enabled */

                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_NOR    /* Normal                     */
                                           /* Not Cacheable              */
                                           /* Not Bufferable             */
                                           /* Not Shareable              */
                                           /* No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_OCRAM_M4_BASE, 1 * 1024 * 1024,

                                           /* Instruction access Enabled */

                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_SO   | /* Strongly Ordered           */
                       RASR_C_VALUE      | /* Cacheable DCACHE ? 0 : 1   */
                       RASR_B_VALUE        /* Bufferable WB    ? 0 : 1   */
                                           /* Not Shareable              */
                                           /* No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_OCRAM_M4_BASE + (1 * 1024 * 1024), 512 * 1024,

                                           /* Instruction access Enabled */

                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_SO   | /* Strongly Ordered           */
                       RASR_C_VALUE      | /* Cacheable DCACHE ? 0 : 1   */
                       RASR_B_VALUE        /* Bufferable WB    ? 0 : 1   */
                                           /* Not Shareable              */
                                           /* No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_FLEXSPI1_CIPHER_BASE, 16 * 1024 * 1024,

                                           /* Instruction access Enabled */

                       MPU_RASR_AP_RORO  | /* P:R0   U:R0                */
                       MPU_RASR_TEX_SO   | /* Strongly Ordered           */
                       MPU_RASR_C        | /* Cacheable                  */
                       MPU_RASR_B          /* Bufferable                 */
                                           /* Not Shareable              */
                                           /* No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_AIPS1_BASE, 16 * 1024 * 1024,

                                           /* Instruction access Enabled */

                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device                     */
                                           /* Not Cacheable              */
                                           /* Not Bufferable             */
                                           /* Not Shareable              */
                                           /* No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_SIM_DISP_BASE, 2 * 1024 * 1024,

                                           /* Instruction access Enabled */

                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device                     */
                                           /* Not Cacheable              */
                                           /* Not Bufferable             */
                                           /* Not Shareable              */
                                           /* No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_SIM_M7_BASE, 1 * 1024 * 1024,

                                           /* Instruction access Enabled */

                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device                     */
                                           /* Not Cacheable              */
                                           /* Not Bufferable             */
                                           /* Not Shareable              */
                                           /* No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_GPU2D_BASE, 2 * 1024 * 1024,

                                           /* Instruction access Enabled */

                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device                     */
                                           /* Not Cacheable              */
                                           /* Not Bufferable             */
                                           /* Not Shareable              */
                                           /* No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_AIPS_M7_BASE, 1 * 1024 * 1024,

                                           /* Instruction access Enabled */

                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device                     */
                                           /* Not Cacheable              */
                                           /* Not Bufferable             */
                                           /* Not Shareable              */
                                           /* No Subregion disable       */
                       );

/****************************************************************************
 * Public Functions
 ****************************************************************************/

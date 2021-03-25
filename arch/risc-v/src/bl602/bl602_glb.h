/****************************************************************************
 * arch/risc-v/src/bl602/bl602_glb.h
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

#ifndef __ARCH_RISCV_SRC_BL602_BL602_GLB_H
#define __ARCH_RISCV_SRC_BL602_BL602_GLB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#define BL602_GLB_EM_0KB  0x0 /* 0KB */
#define BL602_GLB_EM_8KB  0x3 /* 8KB */
#define BL602_GLB_EM_16KB 0xF /* 16KB */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_swrst_ahb_slave1
 *
 * Description:
 *   SW Reset ahb slave.
 *
 * Input Parameters:
 *   slave1: reset signal
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bl602_swrst_ahb_slave1(uint32_t slave1);

/****************************************************************************
 * Name: bl602_glb_get_bclk_div
 *
 * Description:
 *   get bus clock div.
 *
 * Input Parameters:
 *   void
 *
 * Returned Value:
 *   bus clock div
 *
 ****************************************************************************/

uint8_t bl602_glb_get_bclk_div(void);

/****************************************************************************
 * Name: bl602_set_em_sel
 *
 * Description:
 *   Set how much wifi ram is allocated to ble.
 *
 * Input Parameters:
 *   em_type: memory size type
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bl602_set_em_sel(int em_type);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_BL602_BL602_GLB_H */

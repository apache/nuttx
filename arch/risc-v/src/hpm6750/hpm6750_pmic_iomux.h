/****************************************************************************
 * arch/risc-v/src/hpm6750/hpm6750_pmic_iomux.h
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

#ifndef __ARCH_RISCV_SRC_HPM6750_HPM6750_PMIC_IOMUX_H
#define __ARCH_RISCV_SRC_HPM6750_HPM6750_PMIC_IOMUX_H

/* IOC_PY00_FUNC_CTL function mux definitions */
#define IOC_PY00_FUNC_CTL_PGPIO_Y_00           IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0)
#define IOC_PY00_FUNC_CTL_JTAG_TDO             IOC_PAD_FUNC_CTL_ALT_SELECT_SET(1)
#define IOC_PY00_FUNC_CTL_PTMR_COMP_0          IOC_PAD_FUNC_CTL_ALT_SELECT_SET(2)
#define IOC_PY00_FUNC_CTL_SOC_PY_00            IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3)

/* IOC_PY01_FUNC_CTL function mux definitions */
#define IOC_PY01_FUNC_CTL_PGPIO_Y_01           IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0)
#define IOC_PY01_FUNC_CTL_JTAG_TDI             IOC_PAD_FUNC_CTL_ALT_SELECT_SET(1)
#define IOC_PY01_FUNC_CTL_PTMR_COMP_1          IOC_PAD_FUNC_CTL_ALT_SELECT_SET(2)
#define IOC_PY01_FUNC_CTL_SOC_PY_01            IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3)

/* IOC_PY02_FUNC_CTL function mux definitions */
#define IOC_PY02_FUNC_CTL_PGPIO_Y_02           IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0)
#define IOC_PY02_FUNC_CTL_JTAG_TCK             IOC_PAD_FUNC_CTL_ALT_SELECT_SET(1)
#define IOC_PY02_FUNC_CTL_PTMR_COMP_2          IOC_PAD_FUNC_CTL_ALT_SELECT_SET(2)
#define IOC_PY02_FUNC_CTL_SOC_PY_02            IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3)

/* IOC_PY03_FUNC_CTL function mux definitions */
#define IOC_PY03_FUNC_CTL_PGPIO_Y_03           IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0)
#define IOC_PY03_FUNC_CTL_JTAG_TMS             IOC_PAD_FUNC_CTL_ALT_SELECT_SET(1)
#define IOC_PY03_FUNC_CTL_PTMR_COMP_3          IOC_PAD_FUNC_CTL_ALT_SELECT_SET(2)
#define IOC_PY03_FUNC_CTL_SOC_PY_03            IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3)

/* IOC_PY04_FUNC_CTL function mux definitions */
#define IOC_PY04_FUNC_CTL_PGPIO_Y_04           IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0)
#define IOC_PY04_FUNC_CTL_JTAG_TRST            IOC_PAD_FUNC_CTL_ALT_SELECT_SET(1)
#define IOC_PY04_FUNC_CTL_PTMR_COMP_0          IOC_PAD_FUNC_CTL_ALT_SELECT_SET(2)
#define IOC_PY04_FUNC_CTL_SOC_PY_04            IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3)

/* IOC_PY05_FUNC_CTL function mux definitions */
#define IOC_PY05_FUNC_CTL_PGPIO_Y_05           IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0)
#define IOC_PY05_FUNC_CTL_PWDG_RST             IOC_PAD_FUNC_CTL_ALT_SELECT_SET(1)
#define IOC_PY05_FUNC_CTL_PTMR_CAPT_0          IOC_PAD_FUNC_CTL_ALT_SELECT_SET(2)
#define IOC_PY05_FUNC_CTL_SOC_PY_05            IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3)

/* IOC_PY06_FUNC_CTL function mux definitions */
#define IOC_PY06_FUNC_CTL_PGPIO_Y_06           IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0)
#define IOC_PY06_FUNC_CTL_PUART_TXD            IOC_PAD_FUNC_CTL_ALT_SELECT_SET(1)
#define IOC_PY06_FUNC_CTL_PTMR_COMP_1          IOC_PAD_FUNC_CTL_ALT_SELECT_SET(2)
#define IOC_PY06_FUNC_CTL_SOC_PY_06            IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3)

/* IOC_PY07_FUNC_CTL function mux definitions */
#define IOC_PY07_FUNC_CTL_PGPIO_Y_07           IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0)
#define IOC_PY07_FUNC_CTL_PUART_RXD            IOC_PAD_FUNC_CTL_ALT_SELECT_SET(1)
#define IOC_PY07_FUNC_CTL_PTMR_CAPT_1          IOC_PAD_FUNC_CTL_ALT_SELECT_SET(2)
#define IOC_PY07_FUNC_CTL_SOC_PY_07            IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3)

/* IOC_PY08_FUNC_CTL function mux definitions */
#define IOC_PY08_FUNC_CTL_PGPIO_Y_08           IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0)
#define IOC_PY08_FUNC_CTL_PUART_RTS            IOC_PAD_FUNC_CTL_ALT_SELECT_SET(1)
#define IOC_PY08_FUNC_CTL_PTMR_COMP_2          IOC_PAD_FUNC_CTL_ALT_SELECT_SET(2)
#define IOC_PY08_FUNC_CTL_SOC_PY_08            IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3)

/* IOC_PY09_FUNC_CTL function mux definitions */
#define IOC_PY09_FUNC_CTL_PGPIO_Y_09           IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0)
#define IOC_PY09_FUNC_CTL_PUART_CTS            IOC_PAD_FUNC_CTL_ALT_SELECT_SET(1)
#define IOC_PY09_FUNC_CTL_PTMR_CAPT_2          IOC_PAD_FUNC_CTL_ALT_SELECT_SET(2)
#define IOC_PY09_FUNC_CTL_SOC_PY_09            IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3)

/* IOC_PY10_FUNC_CTL function mux definitions */
#define IOC_PY10_FUNC_CTL_PGPIO_Y_10           IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0)
#define IOC_PY10_FUNC_CTL_VAD_CLK              IOC_PAD_FUNC_CTL_ALT_SELECT_SET(1)
#define IOC_PY10_FUNC_CTL_PTMR_COMP_3          IOC_PAD_FUNC_CTL_ALT_SELECT_SET(2)
#define IOC_PY10_FUNC_CTL_SOC_PY_10            IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3)

/* IOC_PY11_FUNC_CTL function mux definitions */
#define IOC_PY11_FUNC_CTL_PGPIO_Y_11           IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0)
#define IOC_PY11_FUNC_CTL_VAD_DAT              IOC_PAD_FUNC_CTL_ALT_SELECT_SET(1)
#define IOC_PY11_FUNC_CTL_PTMR_CAPT_3          IOC_PAD_FUNC_CTL_ALT_SELECT_SET(2)
#define IOC_PY11_FUNC_CTL_SOC_PY_11            IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3)

#endif /* __ARCH_RISCV_SRC_HPM6750_HPM6750_PMIC_IOMUX_H */
/****************************************************************************
 * arch/arm/src/mx8mp/hardware/mx8mp_pinmux.h
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

/* Reference:
 *   "i.MX 8M Plus Applications Processor Reference Manual",
 *   Document Number: IMX8MPRM Rev. 1, 06/2021. NXP
 */

#ifndef __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_PINMUX_H
#define __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_PINMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOMUXC Register Offsets **************************************************/

/* General Purpose Registers */

#define IOMUXC_GPR0_OFFSET                  0x0000
#define IOMUXC_GPR1_OFFSET                  0x0004
#define IOMUXC_GPR2_OFFSET                  0x0008
#define IOMUXC_GPR3_OFFSET                  0x000c
#define IOMUXC_GPR4_OFFSET                  0x0010
#define IOMUXC_GPR5_OFFSET                  0x0014
#define IOMUXC_GPR6_OFFSET                  0x0018
#define IOMUXC_GPR7_OFFSET                  0x001c
#define IOMUXC_GPR8_OFFSET                  0x0020
#define IOMUXC_GPR9_OFFSET                  0x0024
#define IOMUXC_GPR10_OFFSET                 0x0028
#define IOMUXC_GPR11_OFFSET                 0x002c
#define IOMUXC_GPR12_OFFSET                 0x0030
#define IOMUXC_GPR13_OFFSET                 0x0034
#define IOMUXC_GPR14_OFFSET                 0x0038
#define IOMUXC_GPR15_OFFSET                 0x003c
#define IOMUXC_GPR16_OFFSET                 0x0040
#define IOMUXC_GPR17_OFFSET                 0x0044
#define IOMUXC_GPR18_OFFSET                 0x0048
#define IOMUXC_GPR19_OFFSET                 0x004c
#define IOMUXC_GPR20_OFFSET                 0x0050
#define IOMUXC_GPR21_OFFSET                 0x0054
#define IOMUXC_GPR22_OFFSET                 0x0058
#define IOMUXC_GPR23_OFFSET                 0x005c
#define IOMUXC_GPR24_OFFSET                 0x0060

/* The pin function ID is a tuple that contains in order:
 *  mux_register
 *  mux_mode
 *  input_register
 *  input_daisy
 *  config_register)
 */

//format IOMUXC_ | PMUGRF_GPIO or GRF_GPIO | number of the GPIO (0-4) | Bank | IOXX Number | uppercase name on TRM such as `mac_txen` or `uart1bb_sin`
// all iomux appear ot be 2 bits
// PMU GRF GPIO1B[7] 
#define IOMUXC_PMUGRF_GPIO1B_IO07_GPIO
#define IOMUXC_PMUGRF_GPIO1B_IO07_SPI3PMU_RXD
#define IOMUXC_PMUGRF_GPIO1B_IO07_I2C0PMU_SCL

#define IOMUXC_PMUGRF_GPIO1C_IO00_GPIO
#define IOMUXC_PMUGRF_GPIO1C_IO00_SPI3PMU_TXD
#define IOMUXC_PMUGRF_GPIO1C_IO00_I2C0PMU_SCL

#define IOMUXC_GRF_GPIO2B_IO07
#define IOMUXC_GRF_GPIO2C_IO07
#define IOMUXC_GRF_GPIO2D_IO07

#define IOMUXC_GRF_GPIO3A_IO01_GPIO                         GRF_GPIO3A_IOMUX_ADDR, 0b00, 0x00000000, 0x0, 0x00000000
#define IOMUXC_GRF_GPIO3A_IO01_DP_HOTPLUG                   GRF_GPIO3A_IOMUX_ADDR, 0b01, 0x00000000, 0x0, 0x00000000

#define IOMUXC_GRF_GPIO3B_IO02_GPIO
#define IOMUXC_GRF_GPIO3B_IO02_MAC_RXER
#define IOMUXC_GRF_GPIO3B_IO02_I2C5TRACKPAD_SDA


#define IOMUXC_GRF_GPIO3B_IO03_GPIO
#define IOMUXC_GRF_GPIO3B_IO03_MAC_CLK
#define IOMUXC_GRF_GPIO3B_IO03_I2C5TRACKPAD_SCL

#define IOMUXC_GRF_GPIO4A_IO00_GPIO                         GRF_GPIO4A_IOMUX_ADDR, 0b00, 0x00000000, 0x0, 0x00000000
#define IOMUXC_GRF_GPIO4A_IO00_I2S_CLK                      GRF_GPIO4A_IOMUX_ADDR, 0b01, 0x00000000, 0x0, 0x00000000
#define IOMUXC_GRF_GPIO4A_IO00_JTAG_TRACE_CTL               GRF_GPIO4A_IOMUX_ADDR, 0b10, 0x00000000, 0x0, 0x00000000

//todo add offset for pins other than [0]
// GPIO 4C
#define IOMUXC_GRF_GPIO4C_IO00_GPIO                         GRF_GPIO4C_IOMUX_ADDR, 0x0, 0x00000000, 0x0, 0x00000000
#define IOMUXC_GRF_GPIO4C_IO00_UART2DBGD_SIN                GRF_GPIO4C_IOMUX_ADDR, 0x1, 0x00000000, 0x0, 0x00000000
#define IOMUXC_GRF_GPIO4C_IO00_I2C3HDMI_SDA                GRF_GPIO4C_IOMUX_ADDR, 0x2, 0x00000000, 0x0, 0x00000000
#define IOMUXC_GRF_GPIO4C_IO00_HDMII2C_SDA                 GRF_GPIO4C_IOMUX_ADDR, 0x3, 0x00000000, 0x0, 0x00000000

#define IOMUXC_GRF_GPIO4C_IO01_GPIO                         GRF_GPIO4C_IOMUX_ADDR, 0x0, 0x00000000, 0x0, 0x00000000
#define IOMUXC_GRF_GPIO4C_IO01_UART2DBGD_SOUT                GRF_GPIO4C_IOMUX_ADDR, 0x1, 0x00000000, 0x0, 0x00000000
#define IOMUXC_GRF_GPIO4C_IO01_I2C3HDMI_SCL                GRF_GPIO4C_IOMUX_ADDR, 0x2, 0x00000000, 0x0, 0x00000000
#define IOMUXC_GRF_GPIO4C_IO01_HDMII2C_SCL                 GRF_GPIO4C_IOMUX_ADDR, 0x3, 0x00000000, 0x0, 0x00000000

#define IOMUXC_GRF_GPIO4C_IO02_GPIO                 
#define IOMUXC_GRF_GPIO4C_IO02_PWM_0                
#define IOMUXC_GRF_GPIO4C_IO02_VOP0_PWM             
#define IOMUXC_GRF_GPIO4C_IO02_VOP1_PWM             

#define IOMUXC_GRF_GPIO4C_IO03_GPIO                   
#define IOMUXC_GRF_GPIO4C_IO03_UART2DBGC_SIN                  
#define IOMUXC_GRF_GPIO4C_IO03_UARTHDCP_SIN            

#define IOMUXC_GRF_GPIO4C_IO04_GPIO                   
#define IOMUXC_GRF_GPIO4C_IO04_UART2DBGC_SOUT                  
#define IOMUXC_GRF_GPIO4C_IO04_UARTHDCP_SOUT  

#define IOMUXC_GRF_GPIO4C_IO05_GPIO                   
#define IOMUXC_GRF_GPIO4C_IO05_SPDIF_TX       

#define IOMUXC_GRF_GPIO4C_IO06_GPIO                   
#define IOMUXC_GRF_GPIO4C_IO06_PWM_1  

#define IOMUXC_GRF_GPIO4C_IO07_GPIO                   
#define IOMUXC_GRF_GPIO4C_IO07_HDMI_CECINOUT
#define IOMUXC_GRF_GPIO4C_IO07_EDP_HOTPLUG
// GPIO 4D
#define IOMUXC_GRF_GPIO4D_IO00_GPIO         
#define IOMUXC_GRF_GPIO4D_IO00_PCIE_CLKREQNB

#define IOMUXC_GRF_GPIO4D_IO01_GPIO
#define IOMUXC_GRF_GPIO4D_IO01_DP_HOTPLUG 

//////


/* DSE - Drive Strength Field
 *  00   x1
 *  10   x2
 *  01   x4
 *  11   x6
 */
#define PAD_CTL_DSE1            (0 << 1)
#define PAD_CTL_DSE2            (2 << 1)
#define PAD_CTL_DSE4            (1 << 1)
#define PAD_CTL_DSE6            (3 << 1)

/* FSEL - Slew Rate Field
 *  0  Slow Slew Rate (SR=1)
 *  1  Fast Slew Rate (SR=0)
 */
#define PAD_CTL_FSEL            (1 << 4)

/* ODE - Open drain field
 *  0  Disable
 *  1  Enable
 */
#define PAD_CTL_ODE             (1 << 5)

/* PUE - Pull Up / Down Config. Field
 *  0  Weak pull down
 *  1  Weak pull up
 */
#define PAD_CTL_PUE             (1 << 6)

/* HYS - Input Select Field
 *  0  CMOS
 *  1  Schmitt
 */
#define PAD_CTL_HYS             (1 << 7)

/* PE - Pull Select Field
 *  0  Pull Disable
 *  1  Pull Enable
 */
#define PAD_CTL_PE              (1 << 8)

/* Helpers for common configurations */

#define GPIO_PAD_CTRL	(PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PE | PAD_CTL_DSE2)
#define UART_PAD_CTRL	(PAD_CTL_PUE | PAD_CTL_PE)

#endif /* __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_PINMUX_H */

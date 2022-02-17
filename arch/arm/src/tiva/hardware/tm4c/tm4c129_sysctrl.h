/****************************************************************************
 * arch/arm/src/tiva/hardware/tm4c/tm4c129_sysctrl.h
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_TM4C_TM4C129_SYSCTRL_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_TM4C_TM4C129_SYSCTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System Control Register Offsets ******************************************/

/* System Control Registers (System Control Offset) */

#define TIVA_SYSCON_DID0_OFFSET        0x0000 /* Device Identification 0 */
#define TIVA_SYSCON_DID1_OFFSET        0x0004 /* Device Identification 1 */
#define TIVA_SYSCON_PTBOCTL_OFFSET     0x0038 /* Power-Temp Brown Out Control */
#define TIVA_SYSCON_RIS_OFFSET         0x0050 /* Raw Interrupt Status */
#define TIVA_SYSCON_IMC_OFFSET         0x0054 /* Interrupt Mask Control */
#define TIVA_SYSCON_MISC_OFFSET        0x0058 /* Masked Interrupt Status and Clear */
#define TIVA_SYSCON_RESC_OFFSET        0x005c /* Reset Cause */
#define TIVA_SYSCON_PWRTC_OFFSET       0x0060 /* Power-Temperature Cause */
#define TIVA_SYSCON_NMIC_OFFSET        0x0064 /* NMI Cause Register */
#define TIVA_SYSCON_MOSCCTL_OFFSET     0x007c /* Main Oscillator Control */
#define TIVA_SYSCON_RSCLKCFG_OFFSET    0x00b0 /* Run and Sleep Mode Configuration Register */
#define TIVA_SYSCON_MEMTIM0_OFFSET     0x00c0 /* Memory Timing Parameter Register 0 */
#define TIVA_SYSCON_ALTCLKCFG_OFFSET   0x0138 /* Alternate Clock Configuration */
#define TIVA_SYSCON_DSCLKCFG_OFFSET    0x0144 /* Deep Sleep Clock Configuration Register */
#define TIVA_SYSCON_DIVSCLK_OFFSET     0x0148 /* Divisor and Source Clock Configuration */
#define TIVA_SYSCON_SYSPROP_OFFSET     0x014c /* System Properties */
#define TIVA_SYSCON_PIOSCCAL_OFFSET    0x0150 /* Precision Internal Oscillator Calibration */
#define TIVA_SYSCON_PIOSCSTAT_OFFSET   0x0154 /* Precision Internal Oscillator Statistics */
#define TIVA_SYSCON_PLLFREQ0_OFFSET    0x0160 /* PLL Frequency 0 */
#define TIVA_SYSCON_PLLFREQ1_OFFSET    0x0164 /* PLL Frequency 1 */
#define TIVA_SYSCON_PLLSTAT_OFFSET     0x0168 /* PLL Status */
#define TIVA_SYSCON_SLPPWRCFG_OFFSET   0x0188 /* Sleep Power Configuration */
#define TIVA_SYSCON_DSLPPWRCFG_OFFSET  0x018c /* Deep-Sleep Power Configuration */
#define TIVA_SYSCON_NVMSTAT_OFFSET     0x01a0 /* Non-Volatile Memory Information */
#define TIVA_SYSCON_LDOSPCTL_OFFSET    0x01b4 /* LDO Sleep Power Control */
#define TIVA_SYSCON_LDOSPCAL_OFFSET    0x01b8 /* LDO Sleep Power Calibration */
#define TIVA_SYSCON_LDODPCTL_OFFSET    0x01bc /* LDO Deep-Sleep Power Control */
#define TIVA_SYSCON_LDODPCAL_OFFSET    0x01c0 /* LDO Deep-Sleep Power Calibration */
#define TIVA_SYSCON_SDPMST_OFFSET      0x01cc /* Sleep / Deep-Sleep Power Mode Status */
#define TIVA_SYSCON_RESBEHAVCTL_OFFSET 0x01d8 /* Reset Behavior Control Register */
#define TIVA_SYSCON_HSSR_OFFSET        0x01f4 /* Hardware System Service Request */
#define TIVA_SYSCON_USBPDS_OFFSET      0x0280 /* USB Power Domain Status */
#define TIVA_SYSCON_USBMPC_OFFSET      0x0284 /* USB Memory Power Control */
#define TIVA_SYSCON_EMACPDS_OFFSET     0x0288 /* Ethernet MAC Power Domain Status */
#define TIVA_SYSCON_EMACMPC_OFFSET     0x028c /* Ethernet MAC Memory Power Control */
#define TIVA_SYSCON_LCDPDS_OFFSET      0x0290 /* LCD Power Domain Status */
#define TIVA_SYSCON_LCDMPC_OFFSET      0x0294 /* LCD Memory Power Control */
#define TIVA_SYSCON_CAN0PDS_OFFSET     0x0298 /* CAN 0 Power Domain Status */
#define TIVA_SYSCON_CAN0MPC_OFFSET     0x029c /* CAN 0 Memory Power Control */
#define TIVA_SYSCON_CAN1PDS_OFFSET     0x02a0 /* CAN 1 Power Domain Status */
#define TIVA_SYSCON_CAN1MPC_OFFSET     0x02a4 /* CAN 1 Memory Power Control */
#define TIVA_SYSCON_PPWD_OFFSET        0x0300 /* Watchdog Timer Peripheral Present */
#define TIVA_SYSCON_PPTIMER_OFFSET     0x0304 /* 16/32-Bit Timer Peripheral Present */
#define TIVA_SYSCON_PPGPIO_OFFSET      0x0308 /* GPIO Peripheral Present */
#define TIVA_SYSCON_PPDMA_OFFSET       0x030c /* μDMA Peripheral Present */
#define TIVA_SYSCON_PPEPI_OFFSET       0x0310 /* EPI Peripheral Present */
#define TIVA_SYSCON_PPHIB_OFFSET       0x0314 /* Hibernation Peripheral Present */
#define TIVA_SYSCON_PPUART_OFFSET      0x0318 /* UART Peripheral Present */
#define TIVA_SYSCON_PPSSI_OFFSET       0x031c /* SSI Peripheral Present */
#define TIVA_SYSCON_PPI2C_OFFSET       0x0320 /* I2C Peripheral Present */
#define TIVA_SYSCON_PPUSB_OFFSET       0x0328 /* USB Peripheral Present */
#define TIVA_SYSCON_PPEPHY_OFFSET      0x0330 /* Ethernet PHY Peripheral Present */
#define TIVA_SYSCON_PPCAN_OFFSET       0x0334 /* CAN Peripheral Present */
#define TIVA_SYSCON_PPADC_OFFSET       0x0338 /* ADC Peripheral Present */
#define TIVA_SYSCON_PPACMP_OFFSET      0x033c /* ACMP Peripheral Present */
#define TIVA_SYSCON_PPPWM_OFFSET       0x0340 /* PWM Peripheral Present */
#define TIVA_SYSCON_PPQEI_OFFSET       0x0344 /* QE Interface Peripheral Present */
#define TIVA_SYSCON_PPLPC_OFFSET       0x0348 /* Low Pin Count Interface Peripheral Present */
#define TIVA_SYSCON_PPPECI_OFFSET      0x0350 /* Platform Environment Control Interface Peripheral Present */
#define TIVA_SYSCON_PPFAN_OFFSET       0x0354 /* Fan Control Peripheral Present */
#define TIVA_SYSCON_PPEEPROM_OFFSET    0x0358 /* EEPROM Peripheral Present */
#define TIVA_SYSCON_PPWTIMER_OFFSET    0x035c /* 32/64-Bit Wide Timer Peripheral Present */
#define TIVA_SYSCON_PPRTS_OFFSET       0x0370 /* Remote Temperature Sensor Peripheral Present */
#define TIVA_SYSCON_PPCCM_OFFSET       0x0374 /* CRC/Crypto Modules Peripheral Present */
#define TIVA_SYSCON_PPLCD_OFFSET       0x0390 /* LCD Peripheral Present */
#define TIVA_SYSCON_PPOWIRE_OFFSET     0x0398 /* 1-Wire Peripheral Present */
#define TIVA_SYSCON_PPEMAC_OFFSET      0x039c /* Ethernet MAC Peripheral Present */
#define TIVA_SYSCON_PPPRB_OFFSET       0x03a0 /* Power Regulator Bus Peripheral Present */
#define TIVA_SYSCON_PPHIM_OFFSET       0x03a4 /* Human Interface Master Peripheral Present */

#define TIVA_SYSCON_SR_OFFSET          0x0500
#define TIVA_SYSCON_SRWD_OFFSET        0x0500 /* Watchdog Timer Software Reset */
#define TIVA_SYSCON_SRTIMER_OFFSET     0x0504 /* 16/32-Bit Timer Software Reset */
#define TIVA_SYSCON_SRGPIO_OFFSET      0x0508 /* GPIO Software Reset */
#define TIVA_SYSCON_SRDMA_OFFSET       0x050c /* μDMA Software Reset */
#define TIVA_SYSCON_SREPI_OFFSET       0x0510 /* EPI Software Reset */
#define TIVA_SYSCON_SRHIB_OFFSET       0x0514 /* Hibernation Software Reset */
#define TIVA_SYSCON_SRUART_OFFSET      0x0518 /* UART Software Reset */
#define TIVA_SYSCON_SRSSI_OFFSET       0x051c /* SSI Software Reset */
#define TIVA_SYSCON_SRI2C_OFFSET       0x0520 /* I2C Software Reset */
#define TIVA_SYSCON_SRUSB_OFFSET       0x0528 /* USB Software Reset */
#define TIVA_SYSCON_SREPHY_OFFSET      0x0530 /* Ethernet PHY Software Reset */
#define TIVA_SYSCON_SRCAN_OFFSET       0x0534 /* CAN Software Reset */
#define TIVA_SYSCON_SRADC_OFFSET       0x0538 /* ADC Software Reset */
#define TIVA_SYSCON_SRACMP_OFFSET      0x053c /* ACMP Software Reset */
#define TIVA_SYSCON_SRPWM_OFFSET       0x0540 /* PWM Software Reset */
#define TIVA_SYSCON_SRQEI_OFFSET       0x0544 /* QE Interface Software Reset */
#define TIVA_SYSCON_SREEPROM_OFFSET    0x0558 /* EEPROM Software Reset */
#define TIVA_SYSCON_SRWTIMER_OFFSET    0x055c /* 32/64-Bit Wide Timer Software Reset */
#define TIVA_SYSCON_SRCCM_OFFSET       0x0574 /* CRC/Crypto Modules Software Reset */
#define TIVA_SYSCON_SRLCD_OFFSET       0x0590 /* LCD Controller Software Reset */
#define TIVA_SYSCON_SROWIRE_OFFSET     0x0598 /* 1-Wire Software Reset */
#define TIVA_SYSCON_SREMAC_OFFSET      0x059c /* Ethernet MAC Software Reset */

#define TIVA_SYSCON_RCGC_OFFSET        0x0600
#define TIVA_SYSCON_RCGCWD_OFFSET      0x0600 /* Watchdog Timer Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCTIMER_OFFSET   0x0604 /* 16/32-Bit Timer Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCGPIO_OFFSET    0x0608 /* GPIO Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCDMA_OFFSET     0x060c /* μDMA Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCEPI_OFFSET     0x0610 /* EPI Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCHIB_OFFSET     0x0614 /* Hibernation Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCUART_OFFSET    0x0618 /* UART Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCSSI_OFFSET     0x061c /* SSI Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCI2C_OFFSET     0x0620 /* I2C Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCUSB_OFFSET     0x0628 /* USB Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCEPHY_OFFSET    0x0630 /* Ethernet PHY Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCCAN_OFFSET     0x0634 /* CAN RunMode Clock Gating Control */
#define TIVA_SYSCON_RCGCADC_OFFSET     0x0638 /* ADC Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCACMP_OFFSET    0x063c /* ACMP Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCPWM_OFFSET     0x0640 /* PWM Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCQEI_OFFSET     0x0644 /* QE Interface Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCEEPROM_OFFSET  0x0658 /* EEPROM Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCWTIMER_OFFSET  0x065c /* 32/64-Bit Wide Timer Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCCCM_OFFSET     0x0674 /* CRC/CryptoModules Run Mode ClockGating Control */
#define TIVA_SYSCON_RCGCLCD_OFFSET     0x0690 /* LCD Controller Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCOWIRE_OFFSET   0x0698 /* 1-Wire Run Mode Clock Gating Control */
#define TIVA_SYSCON_RCGCEMAC_OFFSET    0x069c /* Ethernet MAC Run Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCWD_OFFSET      0x0700 /* Watchdog Timer Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCTIMER_OFFSET   0x0704 /* 16/32-Bit Timer Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCGPIO_OFFSET    0x0708 /* GPIO Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCDMA_OFFSET     0x070c /* μDMA Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCEPI_OFFSET     0x0710 /* EPI Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCHIB_OFFSET     0x0714 /* Hibernation Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCUART_OFFSET    0x0718 /* UART Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCSSI_OFFSET     0x071c /* SSI Sleep Mode Clock GatingControl */
#define TIVA_SYSCON_SCGCI2C_OFFSET     0x0720 /* I2C Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCUSB_OFFSET     0x0728 /* USB Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCEPHY_OFFSET    0x0730 /* Ethernet PHY Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCCAN_OFFSET     0x0734 /* CAN Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCADC_OFFSET     0x0738 /* ADC Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCACMP_OFFSET    0x073c /* ACMP Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCPWM_OFFSET     0x0740 /* PulseWidthModulator Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCQEI_OFFSET     0x0744 /* QE Interface Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCEEPROM_OFFSET  0x0758 /* EEPROM Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCWTIMER_OFFSET  0x075c /* 32/64-Bit Wide Timer Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCCCM_OFFSET     0x0774 /* CRC/Crypto Modules Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCLCD_OFFSET     0x0790 /* LCD Controller Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCOWIRE_OFFSET   0x0798 /* 1-Wire Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_SCGCEMAC_OFFSET    0x079c /* Ethernet MAC Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCWD_OFFSET      0x0800 /* Watchdog Timer Deep-SleepMode Clock Gating Control */
#define TIVA_SYSCON_DCGCTIMER_OFFSET   0x0804 /* 16/32-Bit Timer Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCGPIO_OFFSET    0x0808 /* GPIO Deep-Sleep Mode Clock */
#define TIVA_SYSCON_DCGCDMA_OFFSET     0x080c /* μDMA Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCEPI_OFFSET     0x0810 /* EPI Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCHIB_OFFSET     0x0814 /* Hibernation Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCUART_OFFSET    0x0818 /* UART Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCSSI_OFFSET     0x081c /* SSI Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCI2C_OFFSET     0x0820 /* I2C Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCUSB_OFFSET     0x0828 /* USB Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCEPHY_OFFSET    0x0830 /* Ethernet PHY Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCCAN_OFFSET     0x0834 /* CAN Deep-SleepMode Clock Gating Control */
#define TIVA_SYSCON_DCGCADC_OFFSET     0x0838 /* ADC Deep-Sleep Mode ClockGating Control */
#define TIVA_SYSCON_DCGCACMP_OFFSET    0x083c /* ACMP Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCPWM_OFFSET     0x0840 /* PWM Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCQEI_OFFSET     0x0844 /* QE Interface Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCEEPROM_OFFSET  0x0858 /* EEPROM Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCWTIMER_OFFSET  0x085c /* 32/64-Bit Wide Timer Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCCCM_OFFSET     0x0874 /* CRC/Crypto Modules Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCLCD_OFFSET     0x0890 /* LCD Controller Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCOWIRE_OFFSET   0x0898 /* 1-Wire Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_DCGCEMAC_OFFSET    0x089c /* Ethernet MAC Deep-Sleep Mode Clock Gating Control */
#define TIVA_SYSCON_PCWD_OFFSET        0x0900 /* Watchdog Timer Power Control */
#define TIVA_SYSCON_PCTIMER_OFFSET     0x0904 /* 16/32-Bit Timer Power Control */
#define TIVA_SYSCON_PCGPIO_OFFSET      0x0908 /* GPIO Power Control */
#define TIVA_SYSCON_PCDMA_OFFSET       0x090c /* μDMA Power Control */
#define TIVA_SYSCON_PCEPI_OFFSET       0x0910 /* External Peripheral Interface Power Control */
#define TIVA_SYSCON_PCHIB_OFFSET       0x0914 /* Hibernation Power Control */
#define TIVA_SYSCON_PCUART_OFFSET      0x0918 /* UART Power Control */
#define TIVA_SYSCON_PCSSI_OFFSET       0x091c /* SSI Power Control */
#define TIVA_SYSCON_PCI2C_OFFSET       0x0920 /* I2C Power Control */
#define TIVA_SYSCON_PCUSB_OFFSET       0x0928 /* USB Power Control */
#define TIVA_SYSCON_PCEPHY_OFFSET      0x0930 /* Ethernet PHY Power Control */
#define TIVA_SYSCON_PCCAN_OFFSET       0x0934 /* CAN Power Control */
#define TIVA_SYSCON_PCADC_OFFSET       0x0938 /* ADC Power Control */
#define TIVA_SYSCON_PCACMP_OFFSET      0x093c /* ACMP Power Control */
#define TIVA_SYSCON_PCPWM_OFFSET       0x0940 /* PWM Power Control */
#define TIVA_SYSCON_PCQEI_OFFSET       0x0944 /* QE Interface Power Control */
#define TIVA_SYSCON_PCEEPROM_OFFSET    0x0958 /* EEPROM Power Control */
#define TIVA_SYSCON_PCCCM_OFFSET       0x0974 /* CRC/Crypto Modules Power Control */
#define TIVA_SYSCON_PCLCD_OFFSET       0x0990 /* LCD Controller Power Control */
#define TIVA_SYSCON_PCOWIRE_OFFSET     0x0998 /* 1-Wire Power Control */
#define TIVA_SYSCON_PCEMAC_OFFSET      0x099c /* Ethernet MAC Power Control */
#define TIVA_SYSCON_PRWD_OFFSET        0x0a00 /* Watchdog Timer Peripheral Ready */
#define TIVA_SYSCON_PRTIMER_OFFSET     0x0a04 /* 16/32-Bit Timer Peripheral Ready */
#define TIVA_SYSCON_PRGPIO_OFFSET      0x0a08 /* GPIO Peripheral Ready */
#define TIVA_SYSCON_PRDMA_OFFSET       0x0a0c /* μDMA Peripheral Ready */
#define TIVA_SYSCON_PREPI_OFFSET       0x0a10 /* EPI Peripheral Ready */
#define TIVA_SYSCON_PRHIB_OFFSET       0x0a14 /* Hibernation Peripheral Ready */
#define TIVA_SYSCON_PRUART_OFFSET      0x0a18 /* UART Peripheral Ready */
#define TIVA_SYSCON_PRSSI_OFFSET       0x0a1c /* SSI Peripheral Ready */
#define TIVA_SYSCON_PRI2C_OFFSET       0x0a20 /* I2C Peripheral Ready */
#define TIVA_SYSCON_PRUSB_OFFSET       0x0a28 /* USB Peripheral Ready */
#define TIVA_SYSCON_PREPHY_OFFSET      0x0a30 /* Ethernet PHY Peripheral Ready */
#define TIVA_SYSCON_PRCAN_OFFSET       0x0a34 /* CAN Peripheral Ready */
#define TIVA_SYSCON_PRADC_OFFSET       0x0a38 /* ADC Peripheral Ready */
#define TIVA_SYSCON_PRACMP_OFFSET      0x0a3c /* ACMP Peripheral Ready */
#define TIVA_SYSCON_PRPWM_OFFSET       0x0a40 /* PWM Peripheral Ready */
#define TIVA_SYSCON_PRQEI_OFFSET       0x0a44 /* QE Interface Peripheral Ready */
#define TIVA_SYSCON_PREEPROM_OFFSET    0x0a58 /* EEPROM Peripheral Ready */
#define TIVA_SYSCON_PRWTIMER_OFFSET    0x0a5c /* 32/64-Bit Wide Timer Peripheral Ready */
#define TIVA_SYSCON_PRCCM_OFFSET       0x0a74 /* CRC/Crypto Modules Peripheral Ready */
#define TIVA_SYSCON_PRLCD_OFFSET       0x0a90 /* LCD Controller Peripheral Ready */
#define TIVA_SYSCON_PROWIRE_OFFSET     0x0a98 /* 1-Wire Peripheral Ready */
#define TIVA_SYSCON_PREMAC_OFFSET      0x0a9c /* Ethernet MAC Peripheral Ready */
#define TIVA_SYSCON_UNIQUEID0_OFFSET   0x0f20 /* Unique ID 0 */
#define TIVA_SYSCON_UNIQUEID1_OFFSET   0x0f24 /* Unique ID 1 */
#define TIVA_SYSCON_UNIQUEID2_OFFSET   0x0f28 /* Unique ID 2 */
#define TIVA_SYSCON_UNIQUEID3_OFFSET   0x0f2c /* Unique ID 3 */

/* CCM System Control Registers (CCM Control Offset) */

#define TIVA_SYSCON_CCMCGREQ_OFFSET    0x0204 /* Cryptographic Modules Clock Gating Request */

/* System Control Legacy Register Offsets ***********************************/

#define TIVA_SYSCON_DC0_OFFSET         0x008 /* Device Capabilities 0 */
#define TIVA_SYSCON_DC1_OFFSET         0x010 /* Device Capabilities 1 */
#define TIVA_SYSCON_DC2_OFFSET         0x014 /* Device Capabilities 2 */
#define TIVA_SYSCON_DC3_OFFSET         0x018 /* Device Capabilities 3 */
#define TIVA_SYSCON_DC4_OFFSET         0x01c /* Device Capabilities 4 */
#define TIVA_SYSCON_DC5_OFFSET         0x020 /* Device Capabilities 5 */
#define TIVA_SYSCON_DC6_OFFSET         0x024 /* Device Capabilities 6 */
#define TIVA_SYSCON_DC7_OFFSET         0x028 /* Device Capabilities 7 */
#define TIVA_SYSCON_DC8_OFFSET         0x02c /* Device Capabilities 8 */

#define TIVA_SYSCON_SRCR0_OFFSET       0x040 /* Software Reset Control 0 */
#define TIVA_SYSCON_SRCR1_OFFSET       0x044 /* Software Reset Control 1 */
#define TIVA_SYSCON_SRCR2_OFFSET       0x048 /* Software Reset Control 2 */

#define TIVA_SYSCON_RCGC0_OFFSET       0x100 /* Run Mode Clock Gating Control Register 0 */
#define TIVA_SYSCON_RCGC1_OFFSET       0x104 /* Run Mode Clock Gating Control Register 1 */
#define TIVA_SYSCON_RCGC2_OFFSET       0x108 /* Run Mode Clock Gating Control Register 2 */

#define TIVA_SYSCON_SCGC0_OFFSET       0x110 /* Sleep Mode Clock Gating Control Register 0 */
#define TIVA_SYSCON_SCGC1_OFFSET       0x114 /* Sleep Mode Clock Gating Control Register 1 */
#define TIVA_SYSCON_SCGC2_OFFSET       0x118 /* Sleep Mode Clock Gating Control Register 2 */

#define TIVA_SYSCON_DCGC0_OFFSET       0x120 /* Deep Sleep Mode Clock Gating Control Register 0 */
#define TIVA_SYSCON_DCGC1_OFFSET       0x124 /* Deep Sleep Mode Clock Gating Control Register 1 */
#define TIVA_SYSCON_DCGC2_OFFSET       0x128 /* Deep Sleep Mode Clock Gating Control Register 2 */

#define TIVA_SYSCON_DC9_OFFSET         0x190 /* Device Capabilities */

/* System Control Register Addresses ****************************************/

/* System Control Registers (System Control Offset) */

#define TIVA_SYSCON_DID0               (TIVA_SYSCON_BASE+TIVA_SYSCON_DID0_OFFSET)
#define TIVA_SYSCON_DID1               (TIVA_SYSCON_BASE+TIVA_SYSCON_DID1_OFFSET)
#define TIVA_SYSCON_PTBOCTL            (TIVA_SYSCON_BASE+TIVA_SYSCON_PTBOCTL_OFFSET)
#define TIVA_SYSCON_RIS                (TIVA_SYSCON_BASE+TIVA_SYSCON_RIS_OFFSET)
#define TIVA_SYSCON_IMC                (TIVA_SYSCON_BASE+TIVA_SYSCON_IMC_OFFSET)
#define TIVA_SYSCON_MISC               (TIVA_SYSCON_BASE+TIVA_SYSCON_MISC_OFFSET)
#define TIVA_SYSCON_RESC               (TIVA_SYSCON_BASE+TIVA_SYSCON_RESC_OFFSET)
#define TIVA_SYSCON_PWRTC              (TIVA_SYSCON_BASE+TIVA_SYSCON_PWRTC_OFFSET)
#define TIVA_SYSCON_NMIC               (TIVA_SYSCON_BASE+TIVA_SYSCON_NMIC_OFFSET)
#define TIVA_SYSCON_MOSCCTL            (TIVA_SYSCON_BASE+TIVA_SYSCON_MOSCCTL_OFFSET)
#define TIVA_SYSCON_RSCLKCFG           (TIVA_SYSCON_BASE+TIVA_SYSCON_RSCLKCFG_OFFSET)
#define TIVA_SYSCON_MEMTIM0            (TIVA_SYSCON_BASE+TIVA_SYSCON_MEMTIM0_OFFSET)
#define TIVA_SYSCON_ALTCLKCFG          (TIVA_SYSCON_BASE+TIVA_SYSCON_ALTCLKCFG_OFFSET)
#define TIVA_SYSCON_DSCLKCFG           (TIVA_SYSCON_BASE+TIVA_SYSCON_DSCLKCFG_OFFSET)
#define TIVA_SYSCON_DIVSCLK            (TIVA_SYSCON_BASE+TIVA_SYSCON_DIVSCLK_OFFSET)
#define TIVA_SYSCON_SYSPROP            (TIVA_SYSCON_BASE+TIVA_SYSCON_SYSPROP_OFFSET)
#define TIVA_SYSCON_PIOSCCAL           (TIVA_SYSCON_BASE+TIVA_SYSCON_PIOSCCAL_OFFSET)
#define TIVA_SYSCON_PIOSCSTAT          (TIVA_SYSCON_BASE+TIVA_SYSCON_PIOSCSTAT_OFFSET)
#define TIVA_SYSCON_PLLFREQ0           (TIVA_SYSCON_BASE+TIVA_SYSCON_PLLFREQ0_OFFSET)
#define TIVA_SYSCON_PLLFREQ1           (TIVA_SYSCON_BASE+TIVA_SYSCON_PLLFREQ1_OFFSET)
#define TIVA_SYSCON_PLLSTAT            (TIVA_SYSCON_BASE+TIVA_SYSCON_PLLSTAT_OFFSET)
#define TIVA_SYSCON_SLPPWRCFG          (TIVA_SYSCON_BASE+TIVA_SYSCON_SLPPWRCFG_OFFSET)
#define TIVA_SYSCON_DSLPPWRCFG         (TIVA_SYSCON_BASE+TIVA_SYSCON_DSLPPWRCFG_OFFSET)
#define TIVA_SYSCON_NVMSTAT            (TIVA_SYSCON_BASE+TIVA_SYSCON_NVMSTAT_OFFSET)
#define TIVA_SYSCON_LDOSPCTL           (TIVA_SYSCON_BASE+TIVA_SYSCON_LDOSPCTL_OFFSET)
#define TIVA_SYSCON_LDOSPCAL           (TIVA_SYSCON_BASE+TIVA_SYSCON_LDOSPCAL_OFFSET)
#define TIVA_SYSCON_LDODPCTL           (TIVA_SYSCON_BASE+TIVA_SYSCON_LDODPCTL_OFFSET)
#define TIVA_SYSCON_LDODPCAL           (TIVA_SYSCON_BASE+TIVA_SYSCON_LDODPCAL_OFFSET)
#define TIVA_SYSCON_SDPMST             (TIVA_SYSCON_BASE+TIVA_SYSCON_SDPMST_OFFSET)
#define TIVA_SYSCON_RESBEHAVCTL        (TIVA_SYSCON_BASE+TIVA_SYSCON_RESBEHAVCTL_OFFSET)
#define TIVA_SYSCON_HSSR               (TIVA_SYSCON_BASE+TIVA_SYSCON_HSSR_OFFSET)
#define TIVA_SYSCON_USBPDS             (TIVA_SYSCON_BASE+TIVA_SYSCON_USBPDS_OFFSET)
#define TIVA_SYSCON_USBMPC             (TIVA_SYSCON_BASE+TIVA_SYSCON_USBMPC_OFFSET)
#define TIVA_SYSCON_EMACPDS            (TIVA_SYSCON_BASE+TIVA_SYSCON_EMACPDS_OFFSET)
#define TIVA_SYSCON_EMACMPC            (TIVA_SYSCON_BASE+TIVA_SYSCON_EMACMPC_OFFSET)
#define TIVA_SYSCON_LCDPDS             (TIVA_SYSCON_BASE+TIVA_SYSCON_LCDPDS_OFFSET)
#define TIVA_SYSCON_LCDMPC             (TIVA_SYSCON_BASE+TIVA_SYSCON_LCDMPC_OFFSET)
#define TIVA_SYSCON_CAN0PDS            (TIVA_SYSCON_BASE+TIVA_SYSCON_CAN0PDS_OFFSET)
#define TIVA_SYSCON_CAN0MPC            (TIVA_SYSCON_BASE+TIVA_SYSCON_CAN0MPC_OFFSET)
#define TIVA_SYSCON_CAN1PDS            (TIVA_SYSCON_BASE+TIVA_SYSCON_CAN1PDS_OFFSET)
#define TIVA_SYSCON_CAN1MPC            (TIVA_SYSCON_BASE+TIVA_SYSCON_CAN1MPC_OFFSET)
#define TIVA_SYSCON_PPWD               (TIVA_SYSCON_BASE+TIVA_SYSCON_PPWD_OFFSET)
#define TIVA_SYSCON_PPTIMER            (TIVA_SYSCON_BASE+TIVA_SYSCON_PPTIMER_OFFSET)
#define TIVA_SYSCON_PPGPIO             (TIVA_SYSCON_BASE+TIVA_SYSCON_PPGPIO_OFFSET)
#define TIVA_SYSCON_PPDMA              (TIVA_SYSCON_BASE+TIVA_SYSCON_PPDMA_OFFSET)
#define TIVA_SYSCON_PPEPI              (TIVA_SYSCON_BASE+TIVA_SYSCON_PPEPI_OFFSET)
#define TIVA_SYSCON_PPHIB              (TIVA_SYSCON_BASE+TIVA_SYSCON_PPHIB_OFFSET)
#define TIVA_SYSCON_PPUART             (TIVA_SYSCON_BASE+TIVA_SYSCON_PPUART_OFFSET)
#define TIVA_SYSCON_PPSSI              (TIVA_SYSCON_BASE+TIVA_SYSCON_PPSSI_OFFSET)
#define TIVA_SYSCON_PPI2C              (TIVA_SYSCON_BASE+TIVA_SYSCON_PPI2C_OFFSET)
#define TIVA_SYSCON_PPUSB              (TIVA_SYSCON_BASE+TIVA_SYSCON_PPUSB_OFFSET)
#define TIVA_SYSCON_PPEPHY             (TIVA_SYSCON_BASE+TIVA_SYSCON_PPEPHY_OFFSET)
#define TIVA_SYSCON_PPCAN              (TIVA_SYSCON_BASE+TIVA_SYSCON_PPCAN_OFFSET)
#define TIVA_SYSCON_PPADC              (TIVA_SYSCON_BASE+TIVA_SYSCON_PPADC_OFFSET)
#define TIVA_SYSCON_PPACMP             (TIVA_SYSCON_BASE+TIVA_SYSCON_PPACMP_OFFSET)
#define TIVA_SYSCON_PPPWM              (TIVA_SYSCON_BASE+TIVA_SYSCON_PPPWM_OFFSET)
#define TIVA_SYSCON_PPQEI              (TIVA_SYSCON_BASE+TIVA_SYSCON_PPQEI_OFFSET)
#define TIVA_SYSCON_PPLPC              (TIVA_SYSCON_BASE+TIVA_SYSCON_PPLPC_OFFSET)
#define TIVA_SYSCON_PPPECI             (TIVA_SYSCON_BASE+TIVA_SYSCON_PPPECI_OFFSET)
#define TIVA_SYSCON_PPFAN              (TIVA_SYSCON_BASE+TIVA_SYSCON_PPFAN_OFFSET)
#define TIVA_SYSCON_PPEEPROM           (TIVA_SYSCON_BASE+TIVA_SYSCON_PPEEPROM_OFFSET)
#define TIVA_SYSCON_PPWTIMER           (TIVA_SYSCON_BASE+TIVA_SYSCON_PPWTIMER_OFFSET)
#define TIVA_SYSCON_PPRTS              (TIVA_SYSCON_BASE+TIVA_SYSCON_PPRTS_OFFSET)
#define TIVA_SYSCON_PPCCM              (TIVA_SYSCON_BASE+TIVA_SYSCON_PPCCM_OFFSET)
#define TIVA_SYSCON_PPLCD              (TIVA_SYSCON_BASE+TIVA_SYSCON_PPLCD_OFFSET)
#define TIVA_SYSCON_PPOWIRE            (TIVA_SYSCON_BASE+TIVA_SYSCON_PPOWIRE_OFFSET)
#define TIVA_SYSCON_PPEMAC             (TIVA_SYSCON_BASE+TIVA_SYSCON_PPEMAC_OFFSET)
#define TIVA_SYSCON_PPPRB              (TIVA_SYSCON_BASE+TIVA_SYSCON_PPPRB_OFFSET)
#define TIVA_SYSCON_PPHIM              (TIVA_SYSCON_BASE+TIVA_SYSCON_PPHIM_OFFSET)

#define TIVA_SYSCON_SR_BASE            (TIVA_SYSCON_BASE+TIVA_SYSCON_SR_OFFSET)
#define TIVA_SYSCON_SRWD               (TIVA_SYSCON_BASE+TIVA_SYSCON_SRWD_OFFSET)
#define TIVA_SYSCON_SRTIMER            (TIVA_SYSCON_BASE+TIVA_SYSCON_SRTIMER_OFFSET)
#define TIVA_SYSCON_SRGPIO             (TIVA_SYSCON_BASE+TIVA_SYSCON_SRGPIO_OFFSET)
#define TIVA_SYSCON_SRDMA              (TIVA_SYSCON_BASE+TIVA_SYSCON_SRDMA_OFFSET)
#define TIVA_SYSCON_SREPI              (TIVA_SYSCON_BASE+TIVA_SYSCON_SREPI_OFFSET)
#define TIVA_SYSCON_SRHIB              (TIVA_SYSCON_BASE+TIVA_SYSCON_SRHIB_OFFSET)
#define TIVA_SYSCON_SRUART             (TIVA_SYSCON_BASE+TIVA_SYSCON_SRUART_OFFSET)
#define TIVA_SYSCON_SRSSI              (TIVA_SYSCON_BASE+TIVA_SYSCON_SRSSI_OFFSET)
#define TIVA_SYSCON_SRI2C              (TIVA_SYSCON_BASE+TIVA_SYSCON_SRI2C_OFFSET)
#define TIVA_SYSCON_SRUSB              (TIVA_SYSCON_BASE+TIVA_SYSCON_SRUSB_OFFSET)
#define TIVA_SYSCON_SREPHY             (TIVA_SYSCON_BASE+TIVA_SYSCON_SREPHY_OFFSET)
#define TIVA_SYSCON_SRCAN              (TIVA_SYSCON_BASE+TIVA_SYSCON_SRCAN_OFFSET)
#define TIVA_SYSCON_SRADC              (TIVA_SYSCON_BASE+TIVA_SYSCON_SRADC_OFFSET)
#define TIVA_SYSCON_SRACMP             (TIVA_SYSCON_BASE+TIVA_SYSCON_SRACMP_OFFSET)
#define TIVA_SYSCON_SRPWM              (TIVA_SYSCON_BASE+TIVA_SYSCON_SRPWM_OFFSET)
#define TIVA_SYSCON_SRQEI              (TIVA_SYSCON_BASE+TIVA_SYSCON_SRQEI_OFFSET)
#define TIVA_SYSCON_SREEPROM           (TIVA_SYSCON_BASE+TIVA_SYSCON_SREEPROM_OFFSET)
#define TIVA_SYSCON_SRWTIMER           (TIVA_SYSCON_BASE+TIVA_SYSCON_SRWTIMER_OFFSET)
#define TIVA_SYSCON_SRCCM              (TIVA_SYSCON_BASE+TIVA_SYSCON_SRCCM_OFFSET)
#define TIVA_SYSCON_SRLCD              (TIVA_SYSCON_BASE+TIVA_SYSCON_SRLCD_OFFSET)
#define TIVA_SYSCON_SROWIRE            (TIVA_SYSCON_BASE+TIVA_SYSCON_SROWIRE_OFFSET)
#define TIVA_SYSCON_SREMAC             (TIVA_SYSCON_BASE+TIVA_SYSCON_SREMAC_OFFSET)

#define TIVA_SYSCON_RCGC_BASE          (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGC_OFFSET)
#define TIVA_SYSCON_RCGCWD             (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCWD_OFFSET)
#define TIVA_SYSCON_RCGCTIMER          (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCTIMER_OFFSET)
#define TIVA_SYSCON_RCGCGPIO           (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCGPIO_OFFSET)
#define TIVA_SYSCON_RCGCDMA            (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCDMA_OFFSET)
#define TIVA_SYSCON_RCGCEPI            (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCEPI_OFFSET)
#define TIVA_SYSCON_RCGCHIB            (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCHIB_OFFSET)
#define TIVA_SYSCON_RCGCUART           (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCUART_OFFSET)
#define TIVA_SYSCON_RCGCSSI            (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCSSI_OFFSET)
#define TIVA_SYSCON_RCGCI2C            (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCI2C_OFFSET)
#define TIVA_SYSCON_RCGCUSB            (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCUSB_OFFSET)
#define TIVA_SYSCON_RCGCEPHY           (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCEPHY_OFFSET)
#define TIVA_SYSCON_RCGCCAN            (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCCAN_OFFSET)
#define TIVA_SYSCON_RCGCADC            (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCADC_OFFSET)
#define TIVA_SYSCON_RCGCACMP           (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCACMP_OFFSET)
#define TIVA_SYSCON_RCGCPWM            (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCPWM_OFFSET)
#define TIVA_SYSCON_RCGCQEI            (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCQEI_OFFSET)
#define TIVA_SYSCON_RCGCEEPROM         (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCEEPROM_OFFSET)
#define TIVA_SYSCON_RCGCWTIMER         (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCWTIMER_OFFSET)
#define TIVA_SYSCON_RCGCCCM            (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCCCM_OFFSET)
#define TIVA_SYSCON_RCGCLCD            (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCLCD_OFFSET)
#define TIVA_SYSCON_RCGCOWIRE          (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCOWIRE_OFFSET)
#define TIVA_SYSCON_RCGCEMAC           (TIVA_SYSCON_BASE+TIVA_SYSCON_RCGCEMAC_OFFSET)
#define TIVA_SYSCON_SCGCWD             (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCWD_OFFSET)
#define TIVA_SYSCON_SCGCTIMER          (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCTIMER_OFFSET)
#define TIVA_SYSCON_SCGCGPIO           (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCGPIO_OFFSET)
#define TIVA_SYSCON_SCGCDMA            (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCDMA_OFFSET)
#define TIVA_SYSCON_SCGCEPI            (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCEPI_OFFSET)
#define TIVA_SYSCON_SCGCHIB            (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCHIB_OFFSET)
#define TIVA_SYSCON_SCGCUART           (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCUART_OFFSET)
#define TIVA_SYSCON_SCGCSSI            (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCSSI_OFFSET)
#define TIVA_SYSCON_SCGCI2C            (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCI2C_OFFSET)
#define TIVA_SYSCON_SCGCUSB            (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCUSB_OFFSET)
#define TIVA_SYSCON_SCGCEPHY           (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCEPHY_OFFSET)
#define TIVA_SYSCON_SCGCCAN            (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCCAN_OFFSET)
#define TIVA_SYSCON_SCGCADC            (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCADC_OFFSET)
#define TIVA_SYSCON_SCGCACMP           (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCACMP_OFFSET)
#define TIVA_SYSCON_SCGCPWM            (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCPWM_OFFSET)
#define TIVA_SYSCON_SCGCQEI            (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCQEI_OFFSET)
#define TIVA_SYSCON_SCGCEEPROM         (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCEEPROM_OFFSET)
#define TIVA_SYSCON_SCGCWTIMER         (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCWTIMER_OFFSET)
#define TIVA_SYSCON_SCGCCCM            (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCCCM_OFFSET)
#define TIVA_SYSCON_SCGCLCD            (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCLCD_OFFSET)
#define TIVA_SYSCON_SCGCOWIRE          (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCOWIRE_OFFSET)
#define TIVA_SYSCON_SCGCEMAC           (TIVA_SYSCON_BASE+TIVA_SYSCON_SCGCEMAC_OFFSET)
#define TIVA_SYSCON_DCGCWD             (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCWD_OFFSET)
#define TIVA_SYSCON_DCGCTIMER          (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCTIMER_OFFSET)
#define TIVA_SYSCON_DCGCGPIO           (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCGPIO_OFFSET)
#define TIVA_SYSCON_DCGCDMA            (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCDMA_OFFSET)
#define TIVA_SYSCON_DCGCEPI            (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCEPI_OFFSET)
#define TIVA_SYSCON_DCGCHIB            (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCHIB_OFFSET)
#define TIVA_SYSCON_DCGCUART           (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCUART_OFFSET)
#define TIVA_SYSCON_DCGCSSI            (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCSSI_OFFSET)
#define TIVA_SYSCON_DCGCI2C            (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCI2C_OFFSET)
#define TIVA_SYSCON_DCGCUSB            (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCUSB_OFFSET)
#define TIVA_SYSCON_DCGCEPHY           (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCEPHY_OFFSET)
#define TIVA_SYSCON_DCGCCAN            (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCCAN_OFFSET)
#define TIVA_SYSCON_DCGCADC            (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCADC_OFFSET)
#define TIVA_SYSCON_DCGCACMP           (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCACMP_OFFSET)
#define TIVA_SYSCON_DCGCPWM            (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCPWM_OFFSET)
#define TIVA_SYSCON_DCGCQEI            (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCQEI_OFFSET)
#define TIVA_SYSCON_DCGCEEPROM         (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCEEPROM_OFFSET)
#define TIVA_SYSCON_DCGCWTIMER         (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCWTIMER_OFFSET)
#define TIVA_SYSCON_DCGCCCM            (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCCCM_OFFSET)
#define TIVA_SYSCON_DCGCLCD            (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCLCD_OFFSET)
#define TIVA_SYSCON_DCGCOWIRE          (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCOWIRE_OFFSET)
#define TIVA_SYSCON_DCGCEMAC           (TIVA_SYSCON_BASE+TIVA_SYSCON_DCGCEMAC_OFFSET)
#define TIVA_SYSCON_PCWD               (TIVA_SYSCON_BASE+TIVA_SYSCON_PCWD_OFFSET)
#define TIVA_SYSCON_PCTIMER            (TIVA_SYSCON_BASE+TIVA_SYSCON_PCTIMER_OFFSET)
#define TIVA_SYSCON_PCGPIO             (TIVA_SYSCON_BASE+TIVA_SYSCON_PCGPIO_OFFSET)
#define TIVA_SYSCON_PCDMA              (TIVA_SYSCON_BASE+TIVA_SYSCON_PCDMA_OFFSET)
#define TIVA_SYSCON_PCEPI              (TIVA_SYSCON_BASE+TIVA_SYSCON_PCEPI_OFFSET)
#define TIVA_SYSCON_PCHIB              (TIVA_SYSCON_BASE+TIVA_SYSCON_PCHIB_OFFSET)
#define TIVA_SYSCON_PCUART             (TIVA_SYSCON_BASE+TIVA_SYSCON_PCUART_OFFSET)
#define TIVA_SYSCON_PCSSI              (TIVA_SYSCON_BASE+TIVA_SYSCON_PCSSI_OFFSET)
#define TIVA_SYSCON_PCI2C              (TIVA_SYSCON_BASE+TIVA_SYSCON_PCI2C_OFFSET)
#define TIVA_SYSCON_PCUSB              (TIVA_SYSCON_BASE+TIVA_SYSCON_PCUSB_OFFSET)
#define TIVA_SYSCON_PCEPHY             (TIVA_SYSCON_BASE+TIVA_SYSCON_PCEPHY_OFFSET)
#define TIVA_SYSCON_PCCAN              (TIVA_SYSCON_BASE+TIVA_SYSCON_PCCAN_OFFSET)
#define TIVA_SYSCON_PCADC              (TIVA_SYSCON_BASE+TIVA_SYSCON_PCADC_OFFSET)
#define TIVA_SYSCON_PCACMP             (TIVA_SYSCON_BASE+TIVA_SYSCON_PCACMP_OFFSET)
#define TIVA_SYSCON_PCPWM              (TIVA_SYSCON_BASE+TIVA_SYSCON_PCPWM_OFFSET)
#define TIVA_SYSCON_PCQEI              (TIVA_SYSCON_BASE+TIVA_SYSCON_PCQEI_OFFSET)
#define TIVA_SYSCON_PCEEPROM           (TIVA_SYSCON_BASE+TIVA_SYSCON_PCEEPROM_OFFSET)
#define TIVA_SYSCON_PCCCM              (TIVA_SYSCON_BASE+TIVA_SYSCON_PCCCM_OFFSET)
#define TIVA_SYSCON_PCLCD              (TIVA_SYSCON_BASE+TIVA_SYSCON_PCLCD_OFFSET)
#define TIVA_SYSCON_PCOWIRE            (TIVA_SYSCON_BASE+TIVA_SYSCON_PCOWIRE_OFFSET)
#define TIVA_SYSCON_PCEMAC             (TIVA_SYSCON_BASE+TIVA_SYSCON_PCEMAC_OFFSET)
#define TIVA_SYSCON_PRWD               (TIVA_SYSCON_BASE+TIVA_SYSCON_PRWD_OFFSET)
#define TIVA_SYSCON_PRTIMER            (TIVA_SYSCON_BASE+TIVA_SYSCON_PRTIMER_OFFSET)
#define TIVA_SYSCON_PRGPIO             (TIVA_SYSCON_BASE+TIVA_SYSCON_PRGPIO_OFFSET)
#define TIVA_SYSCON_PRDMA              (TIVA_SYSCON_BASE+TIVA_SYSCON_PRDMA_OFFSET)
#define TIVA_SYSCON_PREPI              (TIVA_SYSCON_BASE+TIVA_SYSCON_PREPI_OFFSET)
#define TIVA_SYSCON_PRHIB              (TIVA_SYSCON_BASE+TIVA_SYSCON_PRHIB_OFFSET)
#define TIVA_SYSCON_PRUART             (TIVA_SYSCON_BASE+TIVA_SYSCON_PRUART_OFFSET)
#define TIVA_SYSCON_PRSSI              (TIVA_SYSCON_BASE+TIVA_SYSCON_PRSSI_OFFSET)
#define TIVA_SYSCON_PRI2C              (TIVA_SYSCON_BASE+TIVA_SYSCON_PRI2C_OFFSET)
#define TIVA_SYSCON_PRUSB              (TIVA_SYSCON_BASE+TIVA_SYSCON_PRUSB_OFFSET)
#define TIVA_SYSCON_PREPHY             (TIVA_SYSCON_BASE+TIVA_SYSCON_PREPHY_OFFSET)
#define TIVA_SYSCON_PRCAN              (TIVA_SYSCON_BASE+TIVA_SYSCON_PRCAN_OFFSET)
#define TIVA_SYSCON_PRADC              (TIVA_SYSCON_BASE+TIVA_SYSCON_PRADC_OFFSET)
#define TIVA_SYSCON_PRACMP             (TIVA_SYSCON_BASE+TIVA_SYSCON_PRACMP_OFFSET)
#define TIVA_SYSCON_PRPWM              (TIVA_SYSCON_BASE+TIVA_SYSCON_PRPWM_OFFSET)
#define TIVA_SYSCON_PRQEI              (TIVA_SYSCON_BASE+TIVA_SYSCON_PRQEI_OFFSET)
#define TIVA_SYSCON_PREEPROM           (TIVA_SYSCON_BASE+TIVA_SYSCON_PREEPROM_OFFSET)
#define TIVA_SYSCON_PRWTIMER           (TIVA_SYSCON_BASE+TIVA_SYSCON_PRWTIMER_OFFSET)
#define TIVA_SYSCON_PRCCM              (TIVA_SYSCON_BASE+TIVA_SYSCON_PRCCM_OFFSET)
#define TIVA_SYSCON_PRLCD              (TIVA_SYSCON_BASE+TIVA_SYSCON_PRLCD_OFFSET)
#define TIVA_SYSCON_PROWIRE            (TIVA_SYSCON_BASE+TIVA_SYSCON_PROWIRE_OFFSET)
#define TIVA_SYSCON_PREMAC             (TIVA_SYSCON_BASE+TIVA_SYSCON_PREMAC_OFFSET)
#define TIVA_SYSCON_UNIQUEID0          (TIVA_SYSCON_BASE+TIVA_SYSCON_UNIQUEID0_OFFSET)
#define TIVA_SYSCON_UNIQUEID1          (TIVA_SYSCON_BASE+TIVA_SYSCON_UNIQUEID1_OFFSET)
#define TIVA_SYSCON_UNIQUEID2          (TIVA_SYSCON_BASE+TIVA_SYSCON_UNIQUEID2_OFFSET)
#define TIVA_SYSCON_UNIQUEID3          (TIVA_SYSCON_BASE+TIVA_SYSCON_UNIQUEID3_OFFSET)

/* CCM System Control Registers (CCM Control Offset) */

#define TIVA_SYSCON_CCMCGREQ           (TIVA_CCM_BASE+TIVA_SYSCON_CCMCGREQ_OFFSET)

/* System Control Legacy Register Addresses *********************************/

#define TIVA_SYSCON_DC0                (TIVA_SYSCON_BASE + TIVA_SYSCON_DC0_OFFSET)
#define TIVA_SYSCON_DC1                (TIVA_SYSCON_BASE + TIVA_SYSCON_DC1_OFFSET)
#define TIVA_SYSCON_DC2                (TIVA_SYSCON_BASE + TIVA_SYSCON_DC2_OFFSET)
#define TIVA_SYSCON_DC3                (TIVA_SYSCON_BASE + TIVA_SYSCON_DC3_OFFSET)
#define TIVA_SYSCON_DC4                (TIVA_SYSCON_BASE + TIVA_SYSCON_DC4_OFFSET)
#define TIVA_SYSCON_DC5                (TIVA_SYSCON_BASE + TIVA_SYSCON_DC5_OFFSET)
#define TIVA_SYSCON_DC6                (TIVA_SYSCON_BASE + TIVA_SYSCON_DC6_OFFSET)
#define TIVA_SYSCON_DC7                (TIVA_SYSCON_BASE + TIVA_SYSCON_DC7_OFFSET)
#define TIVA_SYSCON_DC8                (TIVA_SYSCON_BASE + TIVA_SYSCON_DC8_OFFSET)

#define TIVA_SYSCON_SRCR0              (TIVA_SYSCON_BASE + TIVA_SYSCON_SRCR0_OFFSET)
#define TIVA_SYSCON_SRCR1              (TIVA_SYSCON_BASE + TIVA_SYSCON_SRCR1_OFFSET)
#define TIVA_SYSCON_SRCR2              (TIVA_SYSCON_BASE + TIVA_SYSCON_SRCR2_OFFSET)

#define TIVA_SYSCON_RCGC0              (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGC0_OFFSET)
#define TIVA_SYSCON_RCGC1              (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGC1_OFFSET)
#define TIVA_SYSCON_RCGC2              (TIVA_SYSCON_BASE + TIVA_SYSCON_RCGC2_OFFSET)

#define TIVA_SYSCON_SCGC0              (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGC0_OFFSET)
#define TIVA_SYSCON_SCGC1              (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGC1_OFFSET)
#define TIVA_SYSCON_SCGC2              (TIVA_SYSCON_BASE + TIVA_SYSCON_SCGC2_OFFSET)

#define TIVA_SYSCON_DCGC0              (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGC0_OFFSET)
#define TIVA_SYSCON_DCGC1              (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGC1_OFFSET)
#define TIVA_SYSCON_DCGC2              (TIVA_SYSCON_BASE + TIVA_SYSCON_DCGC2_OFFSET)

#define TIVA_SYSCON_DC9                (TIVA_SYSCON_BASE + TIVA_SYSCON_DC9_OFFSET)

/* System Control Register Bit Definitions **********************************/

/* System Control Registers (System Control Offset) */

/* Device Identification 0 */

#define SYSCON_DID0_MINOR_SHIFT        0         /* Bits 0-7: Minor Revision of the device */
#define SYSCON_DID0_MINOR_MASK         (0xff << SYSCON_DID0_MINOR_SHIFT)
#  define SYSCON_DID0_MIN_0            (0 << SYSCON_DID0_MINOR_SHIFT) /* Initial device or revision */
#  define SYSCON_DID0_MIN_1            (1 << SYSCON_DID0_MINOR_SHIFT) /* First metal layer change */
#  define SYSCON_DID0_MIN_2            (2 << SYSCON_DID0_MINOR_SHIFT) /* Second metal layer change */

#define SYSCON_DID0_MAJOR_SHIFT        8         /* Bits 8-15: Major Revision of the device */
#define SYSCON_DID0_MAJOR_MASK         (0xff << SYSCON_DID0_MAJOR_SHIFT)
#  define SYSCON_DID0_MAJ_REVA         (0  << SYSCON_DID0_MAJOR_SHIFT) /* Revision A */
#  define SYSCON_DID0_MAJ_REVB         (1 << SYSCON_DID0_MAJOR_SHIFT)  /* Revision B */
#  define SYSCON_DID0_MAJ_REVC         (2 << SYSCON_DID0_MAJOR_SHIFT)  /* Revision C */

#define SYSCON_DID0_CLASS_SHIFT        16        /* Bits 16-23: Device Class */
#define SYSCON_DID0_CLASS_MASK         (0xff << SYSCON_DID0_CLASS_SHIFT)
#  define SYSCON_DID0_CLASS_TM4C123    (5 << SYSCON_DID0_CLASS_SHIFT)  /* Tiva TM4C123x and TM4E123x */
#  define SYSCON_DID0_CLASS_TM4C129    (10 << SYSCON_DID0_CLASS_SHIFT) /* Tiva TM4C129-class */

#define SYSCON_DID0_VER_SHIFT          28        /* Bits 28-30: DID0 Version */
#define SYSCON_DID0_VER_MASK           (7 << SYSCON_DID0_VER_SHIFT)
#  define SYSCON_DID0_VER_1            (1 << SYSCON_DID0_VER_SHIFT) /* Second version of DID0 format */

/* Device Identification 1 */

#define SYSCON_DID1_QUAL_SHIFT         0         /* Bits 1-0: Qualification Status */
#define SYSCON_DID1_QUAL_MASK          (3 << SYSCON_DID1_QUAL_SHIFT)
#  define SYSCON_DID1_QUAL_ES          (0 << SYSCON_DID1_QUAL_SHIFT) /* Engineering Sample */
#  define SYSCON_DID1_QUAL_PP          (1 << SYSCON_DID1_QUAL_SHIFT) /* Pilot Production */
#  define SYSCON_DID1_QUAL_FQ          (2 << SYSCON_DID1_QUAL_SHIFT) /* Fully Qualified */

#define SYSCON_DID1_ROHS               (1 << 2)  /* Bit 2: RoHS-Compliance */
#define SYSCON_DID1_PKG_SHIFT          3         /* Bits 3-4: Package Type */
#define SYSCON_DID1_PKG_MASK           (3 << SYSCON_DID1_PKG_SHIFT)
#  define SYSCON_DID1_PKG_QFP          (1 << SYSCON_DID1_PKG_SHIFT) /* QFP package */
#  define SYSCON_DID1_PKG_BGA          (2 << SYSCON_DID1_PKG_SHIFT) /* BGA package */

#define SYSCON_DID1_TEMP_SHIFT         5         /* Bits 5-7: Temperature Range */
#define SYSCON_DID1_TEMP_MASK          (7 << SYSCON_DID1_TEMP_SHIFT)
#  define SYSCON_DID1_TEMP_C           (0 << SYSCON_DID1_TEMP_SHIFT) /* Commercial temperature */
#  define SYSCON_DID1_TEMP_I           (1 << SYSCON_DID1_TEMP_SHIFT) /* Industrial temperature */
#  define SYSCON_DID1_TEMP_E           (2 << SYSCON_DID1_TEMP_SHIFT) /* Extended temperature */
#  define SYSCON_DID1_TEMP_IE          (3 << SYSCON_DID1_TEMP_SHIFT) /* Industrial and extended */

#define SYSCON_DID1_PINCOUNT_SHIFT     13        /* Bits 13-15: Package Pin Count */
#define SYSCON_DID1_PINCOUNT_MASK      (7 << SYSCON_DID1_PINCOUNT_SHIFT)
#  define SYSCON_DID1_PINCNT_100       (2 << SYSCON_DID1_PINCOUNT_SHIFT)  /* 100-pin LQFP */
#  define SYSCON_DID1_PINCNT_64        (3 << SYSCON_DID1_PINCOUNT_SHIFT)  /* 64-pin LQFP */
#  define SYSCON_DID1_PINCNT_144       (4 << SYSCON_DID1_PINCOUNT_SHIFT)  /* 144-pin LQFP */
#  define SYSCON_DID1_PINCNT_157       (5 << SYSCON_DID1_PINCOUNT_SHIFT)  /* 157-pin BGA */
#  define SYSCON_DID1_PINCNT_128       (6 << SYSCON_DID1_PINCOUNT_SHIFT)  /* 128-pin TQFP */
#  define SYSCON_DID1_PINCNT_1212       (7 << SYSCON_DID1_PINCOUNT_SHIFT) /* 212-pin BGA */

#define SYSCON_DID1_PARTNO_SHIFT       16        /* Bits 16-23: Part Number */
#define SYSCON_DID1_PARTNO_MASK        (0xff << SYSCON_DID1_PARTNO_SHIFT)
#  define SYSCON_DID1_TM4C1294NCPDT    (31 << SYSCON_DID1_PARTNO_SHIFT) /* TM4C1294NCPDT */
#  define SYSCON_DID1_TM4C129XNCZAD    (50 << SYSCON_DID1_PARTNO_SHIFT) /* TM4C129XNCZAD */

#define SYSCON_DID1_FAM_SHIFT          24        /* Bits 24-27: Family */
#define SYSCON_DID1_FAM_MASK           (15 << SYSCON_DID1_FAM_SHIFT)
#  define SYSCON_DID1_FAM_TIVA         (0 << SYSCON_DID1_FAM_SHIFT)  /* Tiva  C family */

#define SYSCON_DID1_VER_SHIFT          28        /* Bits 28-31:  DID1 Version */
#define SYSCON_DID1_VER_MASK           (15 << SYSCON_DID1_VER_SHIFT)
#  define SYSCON_DID1_VER_1            (1 << SYSCON_DID1_VER_SHIFT) /* Second version of DID1 format */

/* Power-Temp Brown Out Control */

#define SYSCON_PTBOCTL_VDD_UBOR_SHIFT     (0)    /* Bits 0-1: VDD (VDDS) under BOR Event Action */
#define SYSCON_PTBOCTL_VDD_UBOR_MASK      (3 << SYSCON_PTBOCTL_VDD_UBOR_SHIFT)
#  define SYSCON_PTBOCTL_VDD_UBOR_NONE    (0 << SYSCON_PTBOCTL_VDD_UBOR_SHIFT) /* No Action */
#  define SYSCON_PTBOCTL_VDD_UBOR_SYSINT  (1 << SYSCON_PTBOCTL_VDD_UBOR_SHIFT) /* System control interrupt */
#  define SYSCON_PTBOCTL_VDD_UBOR_NMI     (2 << SYSCON_PTBOCTL_VDD_UBOR_SHIFT) /* NMI */
#  define SYSCON_PTBOCTL_VDD_UBOR_RST     (3 << SYSCON_PTBOCTL_VDD_UBOR_SHIFT) /* Reset */

#define SYSCON_PTBOCTL_VDDA_UBOR_SHIFT    (8)    /* Bits 8-9: VDDA under BOR Event Action */
#define SYSCON_PTBOCTL_VDDA_UBOR_MASK     (3 << SYSCON_PTBOCTL_VDDA_UBOR_SHIFT)
#  define SYSCON_PTBOCTL_VDDA_UBOR_NONE   (0 << SYSCON_PTBOCTL_VDDA_UBOR_SHIFT) /* No Action */
#  define SYSCON_PTBOCTL_VDDA_UBOR_SYSINT (1 << SYSCON_PTBOCTL_VDDA_UBOR_SHIFT) /* System control interrupt */
#  define SYSCON_PTBOCTL_VDDA_UBOR_NMI    (2 << SYSCON_PTBOCTL_VDDA_UBOR_SHIFT) /* NMI */
#  define SYSCON_PTBOCTL_VDDA_UBOR_RST    (3 << SYSCON_PTBOCTL_VDDA_UBOR_SHIFT) /* Reset */

/* Raw Interrupt Status */

#define SYSCON_RIS_BOR1RIS             (1 << 1)  /* Bit 1:  VDD under BOR1 Raw Interrupt Status */
#define SYSCON_RIS_BORRIS              (1 << 1)  /* Bit 1:  Brown-Out Reset Raw Interrupt Status */
#define SYSCON_RIS_MOFRIS              (1 << 3)  /* Bit 3:  Main Oscillator Failure Raw Interrupt Status */
#define SYSCON_RIS_PLLLRIS             (1 << 6)  /* Bit 6:  PLL Lock Raw Interrupt Status */
#define SYSCON_RIS_USBPLLLRIS          (1 << 7)  /* Bit 7:  USB PLL Lock Raw Interrupt Status */
#define SYSCON_RIS_MOSCPUPRIS          (1 << 8)  /* Bit 8:  MOSC Power Up Raw Interrupt Status */
#define SYSCON_RIS_VDDARIS             (1 << 10) /* Bit 10: VDDA Power OK Event Raw Interrupt Status */
#define SYSCON_RIS_BOR0RIS             (1 << 11) /* Bit 11: VDD under BOR0 Raw Interrupt Status */

/* Interrupt Mask Control */

#define SYSCON_IMC_BOR1IM              (1 << 1)  /* Bit 1:  VDD under BOR1 Interrupt Mask */
#define SYSCON_IMC_BORIM               (1 << 1)  /* Bit 1:  Brown-Out Reset Interrupt Mask */
#define SYSCON_IMC_MOFIM               (1 << 3)  /* Bit 3:  Main Oscillator Failure Interrupt Mask */
#define SYSCON_IMC_PLLLIM              (1 << 6)  /* Bit 6:  PLL Lock Interrupt Mask */
#define SYSCON_IMC_USBPLLLIM           (1 << 7)  /* Bit 7:  USB PLL Lock Interrupt Mask */
#define SYSCON_IMC_MOSCPUPIM           (1 << 8)  /* Bit 8:  MOSC Power Up Interrupt Mask */
#define SYSCON_IMC_VDDAIM              (1 << 10) /* Bit 10: VDDA Power OK Interrupt Mask */
#define SYSCON_IMC_BOR0IM              (1 << 11) /* Bit 11: VDD under BOR0 Interrupt Mask */

/* Masked Interrupt Status and Clear */

#define SYSCON_MISC_BOR1MIS            (1 << 1)  /* Bit 1:  VDD under BOR1 Masked Interrupt Status */
#define SYSCON_MISC_BORMIS             (1 << 1)  /* Bit 1:  BOR Masked Interrupt Status */
#define SYSCON_MISC_MOFMIS             (1 << 3)  /* Bit 3:  Main Oscillator Failure Masked Interrupt Status */
#define SYSCON_MISC_PLLLMIS            (1 << 6)  /* Bit 6:  PLL Lock Masked Interrupt Status */
#define SYSCON_MISC_USBPLLLMIS         (1 << 7)  /* Bit 7:  USB PLL Lock Masked Interrupt Status */
#define SYSCON_MISC_MOSCPUPMIS         (1 << 8)  /* Bit 8:  MOSC Power Up Masked Interrupt Status */
#define SYSCON_MISC_VDDAMIS            (1 << 10) /* Bit 10: VDDA Power OK Masked Interrupt Status */
#define SYSCON_MISC_BOR0MIS            (1 << 11) /* Bit 11: VDD under BOR0 Masked Interrupt Status */

/* Reset Cause */

#define SYSCON_RESC_EXT                (1 << 0)  /* Bit 0:  External Reset */
#define SYSCON_RESC_POR                (1 << 1)  /* Bit 1:  Power-On Reset */
#define SYSCON_RESC_BOR                (1 << 2)  /* Bit 2:  Brown-Out Reset */
#define SYSCON_RESC_WDT0               (1 << 3)  /* Bit 3:  Watchdog Timer 0 Reset */
#define SYSCON_RESC_SW                 (1 << 4)  /* Bit 4:  Software Reset */
#define SYSCON_RESC_WDT1               (1 << 5)  /* Bit 5:  Watchdog Timer 1 Reset */
#define SYSCON_RESC_HIB                (1 << 6)  /* Bit 6:  HIB Reset */
#define SYSCON_RESC_HSSR               (1 << 12) /* Bit 12: HSSR Reset */
#define SYSCON_RESC_MOSCFAIL           (1 << 16) /* Bit 16: MOSC Failure Reset */

/* Power-Temperature Cause */

#define SYSCON_PWRTC_VDD_UBOR          (1 << 0)  /* Bit 0:  VDD Under BOR Status */
#define SYSCON_PWRTC_VDDA_UBOR         (1 << 4)  /* Bit 4:  VDDA Under BOR Status */

/* NMI Cause Register */

#define SYSCON_NMIC_EXTERNAL           (1 << 0)  /* Bit 0:  External Pin NMI */
#define SYSCON_NMIC_POWER              (1 << 2)  /* Bit 2:  Power/Brown Out Event NMI */
#define SYSCON_NMIC_WDT0               (1 << 3)  /* Bit 3:  Watch Dog Timer (WDT) 0 NMI */
#define SYSCON_NMIC_WDT1               (1 << 5)  /* Bit 5:  Watch Dog Timer (WDT) 1 NMI */
#define SYSCON_NMIC_TAMPER             (1 << 9)  /* Bit 9:  Tamper Event NMI */
#define SYSCON_NMIC_MOSCFAIL           (1 << 16) /* Bit 16: MOSC Failure NMI */

/* Main Oscillator Control */

#define SYSCON_MOSCCTL_CVAL            (1 << 0)  /* Bit 0:  Clock Validation for MOSC */
#define SYSCON_MOSCCTL_MOSCIM          (1 << 1)  /* Bit 1:  MOSC Failure Action */
#define SYSCON_MOSCCTL_NOXTAL          (1 << 2)  /* Bit 2:  No Crystal Connected */
#define SYSCON_MOSCCTL_PWRDN           (1 << 3)  /* Bit 3:  Power Down */
#define SYSCON_MOSCCTL_OSCRNG          (1 << 4)  /* Bit 4:  Oscillator Range */

/* Run and Sleep Mode Configuration Register */

#define SYSCON_RSCLKCFG_PSYSDIV_SHIFT   (0)       /* Bits 0-9:PLL System Clock Divisor */
#define SYSCON_RSCLKCFG_PSYSDIV_MASK    (0x3ff << SYSCON_RSCLKCFG_PSYSDIV_SHIFT)
#  define SYSCON_RSCLKCFG_PSYSDIV(n)    ((uint32_t)(n) << SYSCON_RSCLKCFG_PSYSDIV_SHIFT)

#define SYSCON_RSCLKCFG_OSYSDIV_SHIFT   (10)      /* Bits 10-19: Oscillator System Clock Divisor */
#define SYSCON_RSCLKCFG_OSYSDIV_MASK    (0x3ff << SYSCON_RSCLKCFG_OSYSDIV_SHIFT)
#  define SYSCON_RSCLKCFG_OSYSDIV(n)    ((uint32_t)(n) << SYSCON_RSCLKCFG_OSYSDIV_SHIFT)

#define SYSCON_RSCLKCFG_OSCSRC_SHIFT    (20)      /* Bits 20-23: Oscillator Source */
#define SYSCON_RSCLKCFG_OSCSRC_MASK     (15 << SYSCON_RSCLKCFG_OSCSRC_SHIFT)
#  define SYSCON_RSCLKCFG_OSCSRC_PIOSC  (0 << SYSCON_RSCLKCFG_OSCSRC_SHIFT) /* PIOSC is source */
#  define SYSCON_RSCLKCFG_OSCSRC_LFIOSC (1 << SYSCON_RSCLKCFG_OSCSRC_SHIFT) /* LFIOSC is source */
#  define SYSCON_RSCLKCFG_OSCSRC_MOSC   (3 << SYSCON_RSCLKCFG_OSCSRC_SHIFT) /* MOSC is source */
#  define SYSCON_RSCLKCFG_OSCSRC_RTC    (4 << SYSCON_RSCLKCFG_OSCSRC_SHIFT) /* RTCOSC is source */

#define SYSCON_RSCLKCFG_PLLSRC_SHIFT    (24)      /* Bits 24-27: PLL Source */
#define SYSCON_RSCLKCFG_PLLSRC_MASK     (15 << SYSCON_RSCLKCFG_PLLSRC_SHIFT)
#  define SYSCON_RSCLKCFG_PLLSRC_PIOSC  (0 << SYSCON_RSCLKCFG_PLLSRC_SHIFT) /* PIOSC is clock source */
#  define SYSCON_RSCLKCFG_PLLSRC_MOSC   (3 << SYSCON_RSCLKCFG_PLLSRC_SHIFT) /* MOSC is the clock source */

#define SYSCON_RSCLKCFG_USEPLL          (1 << 28) /* Bit 28: Use PLL */
#define SYSCON_RSCLKCFG_ACG             (1 << 29) /* Bit 29: Auto Clock Gating */
#define SYSCON_RSCLKCFG_NEWFREQ         (1 << 30) /* Bit 30: New PLLFREQ Accept */
#define SYSCON_RSCLKCFG_MEMTIMU         (1 << 31) /* Bit 31: Memory Timing Register Update */

/* Memory Timing Parameter Register 0 */

#define SYSCON_MEMTIM0_FWS_SHIFT       (0)       /* Bits 0-3: Flash Wait State */
#define SYSCON_MEMTIM0_FWS_MASK        (15 << SYSCON_MEMTIM0_FWS_SHIFT)
#  define SYSCON_MEMTIM0_FWS(n)        ((uint32_t)(n) << SYSCON_MEMTIM0_FWS_SHIFT)

#define SYSCON_MEMTIM0_FBCE            (1 << 5)  /* Bit 5: Flash Bank Clock Edge */
#define SYSCON_MEMTIM0_FBCHT_SHIFT     (6)       /* Bits 6-9: Flash Bank Clock High Time */
#define SYSCON_MEMTIM0_FBCHT_MASK      (15 << SYSCON_MEMTIM0_FBCHT_SHIFT)
#  define SYSCON_MEMTIM0_FBCHT_0p5     (0 << SYSCON_MEMTIM0_FBCHT_SHIFT) /* 1/2 system clock period */
#  define SYSCON_MEMTIM0_FBCHT_1       (1 << SYSCON_MEMTIM0_FBCHT_SHIFT) /* 1 system clock period */
#  define SYSCON_MEMTIM0_FBCHT_1p5     (2 << SYSCON_MEMTIM0_FBCHT_SHIFT) /* 1.5 system clock periods */
#  define SYSCON_MEMTIM0_FBCHT_2       (3 << SYSCON_MEMTIM0_FBCHT_SHIFT) /* 2 system clock periods */
#  define SYSCON_MEMTIM0_FBCHT_2p5     (4 << SYSCON_MEMTIM0_FBCHT_SHIFT) /* 2.5 system clock periods */
#  define SYSCON_MEMTIM0_FBCHT_3       (5 << SYSCON_MEMTIM0_FBCHT_SHIFT) /* 3 system clock periods */
#  define SYSCON_MEMTIM0_FBCHT_3p5     (6 << SYSCON_MEMTIM0_FBCHT_SHIFT) /* 3.5 system clock periods */
#  define SYSCON_MEMTIM0_FBCHT_4       (7 << SYSCON_MEMTIM0_FBCHT_SHIFT) /* 4 system clock periods */
#  define SYSCON_MEMTIM0_FBCHT_4p5     (8 << SYSCON_MEMTIM0_FBCHT_SHIFT) /* 4.5 system clock periods */

#define SYSCON_MEMTIM0_EWS_SHIFT       (16)      /* Bits 16-19: EEPROM Wait States */
#define SYSCON_MEMTIM0_EWS_MASK        (15 << SYSCON_MEMTIM0_EWS_SHIFT)
#  define SYSCON_MEMTIM0_EWS(n)        ((uint32_t)(n) << SYSCON_MEMTIM0_EWS_SHIFT)

#define SYSCON_MEMTIM0_EBCE            (1 << 21) /* Bit 21: EEPROM Bank Clock Edge */
#define SYSCON_MEMTIM0_EBCHT_SHIFT     (22)      /* Bits 22-25: EEPROM Clock High Time */
#define SYSCON_MEMTIM0_EBCHT_MASK      (15 << SYSCON_MEMTIM0_EBCHT_SHIFT)
#  define SYSCON_MEMTIM0_EBCHT_0p5     (0 << SYSCON_MEMTIM0_EBCHT_SHIFT) /* 1/2 system clock period */
#  define SYSCON_MEMTIM0_EBCHT_1       (1 << SYSCON_MEMTIM0_EBCHT_SHIFT) /* 1 system clock period */
#  define SYSCON_MEMTIM0_EBCHT_1p5     (2 << SYSCON_MEMTIM0_EBCHT_SHIFT) /* 1.5 system clock periods */
#  define SYSCON_MEMTIM0_EBCHT_2       (3 << SYSCON_MEMTIM0_EBCHT_SHIFT) /* 2 system clock periods */
#  define SYSCON_MEMTIM0_EBCHT_2p5     (4 << SYSCON_MEMTIM0_EBCHT_SHIFT) /* 2.5 system clock periods */
#  define SYSCON_MEMTIM0_EBCHT_3       (5 << SYSCON_MEMTIM0_EBCHT_SHIFT) /* 3 system clock periods */
#  define SYSCON_MEMTIM0_EBCHT_3p5     (6 << SYSCON_MEMTIM0_EBCHT_SHIFT) /* 3.5 system clock periods */
#  define SYSCON_MEMTIM0_EBCHT_4       (7 << SYSCON_MEMTIM0_EBCHT_SHIFT) /* 4 system clock periods */
#  define SYSCON_MEMTIM0_EBCHT_4p5     (8 << SYSCON_MEMTIM0_EBCHT_SHIFT) /* 4.5 system clock periods */

#define SYSCON_MEMTIM0_MB1             ((1 << 4) | (1 << 20)) /* Must be one */

/* Alternate Clock Configuration */

#define SYSCON_ALTCLKCFG_ALTCLK_SHIFT    (0)       /* Bits 0-3: Alternate Clock Source */
#define SYSCON_ALTCLKCFG_ALTCLK_MASK     (15 << SYSCON_ALTCLKCFG_ALTCLK_SHIFT)
#  define SYSCON_ALTCLKCFG_ALTCLK_PIOSC  (0 << SYSCON_ALTCLKCFG_ALTCLK_SHIFT) /* PIOSC */
#  define SYSCON_ALTCLKCFG_ALTCLK_RTCOSC (3 << SYSCON_ALTCLKCFG_ALTCLK_SHIFT) /* RTCOSC */
#  define SYSCON_ALTCLKCFG_ALTCLK_LFIOSC (4 << SYSCON_ALTCLKCFG_ALTCLK_SHIFT) /* LFIOSC */

/* Deep Sleep Clock Configuration Register */

#define SYSCON_DSCLKCFG_DSSYSDIV_SHIFT    (0)       /* Bits 0-9: Deep Sleep Clock Divisor */
#define SYSCON_DSCLKCFG_DSSYSDIV_MASK     (0x3ff << SYSCON_DSCLKCFG_DSSYSDIV_SHIFT)
#  define SYSCON_DSCLKCFG_DSSYSDIV(n)     ((uint32_t)(n) << SYSCON_DSCLKCFG_DSSYSDIV_SHIFT)

#define SYSCON_DSCLKCFG_DSOSCSRC_SHIFT    (20)      /* Bits 20-23: Deep Sleep Oscillator Source */
#define SYSCON_DSCLKCFG_DSOSCSRC_MASK     (15 << SYSCON_DSCLKCFG_DSOSCSRC_SHIFT)
#  define SYSCON_DSCLKCFG_DSOSCSRC_PIOSC  (0 << SYSCON_DSCLKCFG_DSOSCSRC_SHIFT) /* PIOSC */
#  define SYSCON_DSCLKCFG_DSOSCSRC_LFIOSC (2 << SYSCON_DSCLKCFG_DSOSCSRC_SHIFT) /* LFIOSC */
#  define SYSCON_DSCLKCFG_DSOSCSRC_MOSC   (3 << SYSCON_DSCLKCFG_DSOSCSRC_SHIFT) /* MOSC */
#  define SYSCON_DSCLKCFG_DSOSCSRC_RTC    (4 << SYSCON_DSCLKCFG_DSOSCSRC_SHIFT) /* RTCOSC */

#define SYSCON_DSCLKCFG_MOSCDPD           (1 << 30) /* Bit 30: MOSC Disable Power Down */
#define SYSCON_DSCLKCFG_PIOSCPD           (1 << 31) /* Bit 31: PIOSC Power Down */

/* Divisor and Source Clock Configuration */

#define SYSCON_DIVSCLK_DIV_SHIFT       (0)       /* Bits 0-7: Divisor Value */
#define SYSCON_DIVSCLK_DIV_MASK        (0xff << SYSCON_DIVSCLK_DIV_SHIFT)
#  define SYSCON_DIVSCLK_DIV(n)        ((uint32_t)(n) << SYSCON_DIVSCLK_DIV_SHIFT)

#define SYSCON_DIVSCLK_SRC_SHIFT       (16)      /* Bits 16-17: Clock Source */
#define SYSCON_DIVSCLK_SRC_MASK        (3 << SYSCON_DIVSCLK_SRC_SHIFT)
#  define SYSCON_DIVSCLK_SRC_SYSCLK    (0 << SYSCON_DIVSCLK_SRC_SHIFT) /* System Clock */
#  define SYSCON_DIVSCLK_SRC_PIOSC     (1 << SYSCON_DIVSCLK_SRC_SHIFT) /* PIOSC */
#  define SYSCON_DIVSCLK_SRC_MOSC      (2 << SYSCON_DIVSCLK_SRC_SHIFT) /* MOSC */

#define SYSCON_DIVSCLK_EN              (1 << 31) /* Bit31: DIVSCLK Enable */

/* System Properties */

#define SYSCON_SYSPROP_FPU             (1 << 0)  /* Bit 0:  FPU Present */
#define SYSCON_SYSPROP_LDOSEQ          (1 << 5)  /* Bit 5:  Automatic LDO Sequence Control Present */
#define SYSCON_SYSPROP_FLASHLPM        (1 << 8)  /* Bit 8:  Flash Memory Sleep/Deep-Sleep Low Power Mode Present */
#define SYSCON_SYSPROP_SRAMLPM         (1 << 10) /* Bit 10: SRAM Sleep/Deep-Sleep Low Power Mode Present */
#define SYSCON_SYSPROP_SRAMSM          (1 << 11) /* Bit 11: SRAM Sleep/Deep-Sleep Standby Mode Present */
#define SYSCON_SYSPROP_PIOSCPDE        (1 << 12) /* Bit 12: PIOSC Power Down Present */
#define SYSCON_SYSPROP_TSPDE           (1 << 16) /* Bit 16: Temp Sense Power Down Enable */
#define SYSCON_SYSPROP_LDOSME          (1 << 17) /* Bit 17: LDO Sleep Mode Enable */

/* Precision Internal Oscillator Calibration */

#define SYSCON_PIOSCCAL_UT_SHIFT       (0)      /* Bits 0-6: User Trim Value */
#define SYSCON_PIOSCCAL_UT_MASK        (0x7f << SYSCON_PIOSCCAL_UT_SHIFT)
#  define SYSCON_PIOSCCAL_UT(n)        ((uint32_t)(n) << SYSCON_PIOSCCAL_UT_SHIFT)

#define SYSCON_PIOSCCAL_UPDATE         (1 << 8)  /* Bit 8:  Update Trim */
#define SYSCON_PIOSCCAL_CAL            (1 << 9)  /* Bit 9:  Start Calibration */
#define SYSCON_PIOSCCAL_UTEN           (1 << 31) /* Bit 31: Use User Trim Value */

/* Precision Internal Oscillator Statistics */

#define SYSCON_PIOSCSTAT_CT_SHIFT      (0)       /* Bits 0-6: Calibration Trim Value */
#define SYSCON_PIOSCSTAT_CT_MASK       (0x7f << SYSCON_PIOSCSTAT_CT_SHIFT)
#  define SYSCON_PIOSCSTAT_CT(n)       ((uint32_t)(n) << SYSCON_PIOSCSTAT_CT_SHIFT)

#define SYSCON_PIOSCSTAT_RESULT_SHIFT  (8)       /* Bits 8-9: Calibration Result */
#define SYSCON_PIOSCSTAT_RESULT_MASK   (3 << SYSCON_PIOSCSTAT_RESULT_SHIFT)
#  define SYSCON_PIOSCSTAT_RESULT(n)   ((uint32_t)(n) << SYSCON_PIOSCSTAT_RESULT_SHIFT)
#  define SYSCON_PIOSCSTAT_CRNONE      (0 << SYSCON_PIOSCSTAT_RESULT_SHIFT)
#  define SYSCON_PIOSCSTAT_CRPASS      (1 << SYSCON_PIOSCSTAT_RESULT_SHIFT)
#  define SYSCON_PIOSCSTAT_CRFAIL      (2 << SYSCON_PIOSCSTAT_RESULT_SHIFT)

#define SYSCON_PIOSCSTAT_DT_SHIFT      (16)      /* Bits 16-22: Default Trim Value */
#define SYSCON_PIOSCSTAT_DT_MASK       (0x7f << SYSCON_PIOSCSTAT_DT_SHIFT)
#  define SYSCON_PIOSCSTAT_DT(n)       ((uint32_t)(n) << SYSCON_PIOSCSTAT_DT_SHIFT)

/* PLL Frequency 0 */

#define SYSCON_PLLFREQ0_MINT_SHIFT     (0)       /* Bits 0-9: PLL M Integer Value */
#define SYSCON_PLLFREQ0_MINT_MASK      (0x3ff << SYSCON_PLLFREQ0_MINT_SHIFT)
#  define SYSCON_PLLFREQ0_MINT(n)      ((uint32_t)(n) << SYSCON_PLLFREQ0_MINT_SHIFT)

#define SYSCON_PLLFREQ0_MFRAC_SHIFT    (10)      /* Bits 10-19:  PLL M Fractional Value */
#define SYSCON_PLLFREQ0_MFRAC_MASK     (0x3ff << SYSCON_PLLFREQ0_MFRAC_SHIFT)
#  define SYSCON_PLLFREQ0_MFRAC(n)     ((uint32_t)(n) << SYSCON_PLLFREQ0_MFRAC_SHIFT)

#define SYSCON_PLLFREQ0_PLLPWR         (1 << 23) /* Bit 23: PLL Power */

/* PLL Frequency 1 */

#define SYSCON_PLLFREQ1_N_SHIFT        (0)       /* Bits 0-4: PLL N Value */
#define SYSCON_PLLFREQ1_N_MASK         (31 << SYSCON_PLLFREQ1_N_SHIFT)
#  define SYSCON_PLLFREQ1_N(n)         ((uint32_t)(n) << SYSCON_PLLFREQ1_N_SHIFT)

#define SYSCON_PLLFREQ1_Q_SHIFT        (8)       /* Bits 8-12: PLL Q Value */
#define SYSCON_PLLFREQ1_Q_MASK         (31 << SYSCON_PLLFREQ1_Q_SHIFT)
#  define SYSCON_PLLFREQ1_Q(n)         ((uint32_t)(n) << SYSCON_PLLFREQ1_Q_SHIFT)

/* PLL Status */

#define SYSCON_PLLSTAT_LOCK            (1 << 0)  /* Bit 0: PLL Lock */

/* Sleep Power Configuration */

#define SYSCON_SLPPWRCFG_SRAMPM_SHIFT      (0)  /* Bits 1-0: SRAM Power Modes */
#define SYSCON_SLPPWRCFG_SRAMPM_MASK       (3 << SYSCON_SLPPWRCFG_SRAMPM_SHIFT)
#  define SYSCON_SLPPWRCFG_SRAMPM_ACTIVE   (0 << SYSCON_SLPPWRCFG_SRAMPM_SHIFT) /* Active Mode */
#  define SYSCON_SLPPWRCFG_SRAMPM_STANDBY  (1 << SYSCON_SLPPWRCFG_SRAMPM_SHIFT) /* Standby Mode */
#  define SYSCON_SLPPWRCFG_SRAMPM_LOWPWR   (3 << SYSCON_SLPPWRCFG_SRAMPM_SHIFT) /* Low Power Mode */

#define SYSCON_SLPPWRCFG_FLASHPM_SHIFT     (4)  /* Bits 5-4: Flash Power Modes */
#define SYSCON_SLPPWRCFG_FLASHPM_MASK      (3 << SYSCON_SLPPWRCFG_FLASHPM_SHIFT)
#  define SYSCON_SLPPWRCFG_FLASHPM_ACTIVE  (0 << SYSCON_SLPPWRCFG_FLASHPM_SHIFT) /* Active Mode */
#  define SYSCON_SLPPWRCFG_FLASHPM_LOWPWRR (2 << SYSCON_SLPPWRCFG_FLASHPM_SHIFT) /* Low Power Mode */

/* Deep-Sleep Power Configuration */

#define SYSCON_DSLPPWRCFG_SRAMPM_SHIFT     (0)  /* Bits 1-0: SRAM Power Modes */
#define SYSCON_DSLPPWRCFG_SRAMPM_MASK      (3 << SYSCON_DSLPPWRCFG_SRAMPM_SHIFT)
#  define SYSCON_DSLPPWRCFG_SRAMPM_ACTIVE  (0 << SYSCON_DSLPPWRCFG_SRAMPM_SHIFT) /* Active Mode */
#  define SYSCON_DSLPPWRCFG_SRAMPM_STANDBY (1 << SYSCON_DSLPPWRCFG_SRAMPM_SHIFT) /* Standby Mode */
#  define SYSCON_DSLPPWRCFG_SRAMPM_LOWPWR  (3 << SYSCON_DSLPPWRCFG_SRAMPM_SHIFT) /* Low Power Mode */

#define SYSCON_DSLPPWRCFG_FLASHPM_SHIFT    (4)  /* Bits 5-4: Flash Power Modes */
#define SYSCON_DSLPPWRCFG_FLASHPM_MASK     (3 << SYSCON_DSLPPWRCFG_FLASHPM_SHIFT)
#  define SYSCON_DSLPPWRCFG_FLASHPM_ACTIVE (0 << SYSCON_DSLPPWRCFG_FLASHPM_SHIFT) /* Active Mode */
#  define SYSCON_DSLPPWRCFG_FLASHPM_LOWPWR (2 << SYSCON_DSLPPWRCFG_FLASHPM_SHIFT) /* Low Power Mode */

#define SYSCON_DSLPPWRCFG_TSPD             (1 << 8)  /* Bit 8:  Temperature Sense Power Down */
#define SYSCON_DSLPPWRCFG_LDOSM            (1 << 9)  /* Bit 9:  LDO Sleep Mode */

/* Non-Volatile Memory Information */

#define TIVA_SYSCON_NVMSTAT_FWB        (1 << 0)  /* Bit 0: 32 Word Flash Write Buffer Available */

/* LDO Sleep Power Control */

#define SYSCON_LDOSPCTL_VLDO_SHIFT     (0)       /* Bits 7-0: LDO Output Voltage */
#define SYSCON_LDOSPCTL_VLDO_MASK      (0xff << SYSCON_LDOSPCTL_VLDO_SHIFT)
#  define SYSCON_LDOSPCTL_VLDO_0p90V   (0x12 << SYSCON_LDOSPCTL_VLDO_SHIFT) /* 0.90 V */
#  define SYSCON_LDOSPCTL_VLDO_0p95V   (0x13 << SYSCON_LDOSPCTL_VLDO_SHIFT) /* 0.95 V */
#  define SYSCON_LDOSPCTL_VLDO_1p00V   (0x14 << SYSCON_LDOSPCTL_VLDO_SHIFT) /* 1.00 V */
#  define SYSCON_LDOSPCTL_VLDO_1p05V   (0x15 << SYSCON_LDOSPCTL_VLDO_SHIFT) /* 1.05 V */
#  define SYSCON_LDOSPCTL_VLDO_1p10V   (0x16 << SYSCON_LDOSPCTL_VLDO_SHIFT) /* 1.10 V */
#  define SYSCON_LDOSPCTL_VLDO_1p15V   (0x17 << SYSCON_LDOSPCTL_VLDO_SHIFT) /* 1.15 V */
#  define SYSCON_LDOSPCTL_VLDO_1p20V   (0x18 << SYSCON_LDOSPCTL_VLDO_SHIFT) /* 1.20 V */

#define SYSCON_LDOSPCTL_VADJEN         (1 << 31) /* Bit 31: Voltage Adjust Enable */

/* LDO Sleep Power Calibration */

#define SYSCON_LDOSPCAL_NOPLL_SHIFT     (0)     /* Bits 7-0: Sleep without PLL */
#define SYSCON_LDOSPCAL_NOPLL_MASK      (0xff << SYSCON_LDOSPCAL_NOPLL_SHIFT)
#  define SYSCON_LDOSPCAL_NOPLL_0p90V   (0x12 << SYSCON_LDOSPCAL_NOPLL_SHIFT) /* 0.90 V */
#  define SYSCON_LDOSPCAL_NOPLL_0p95V   (0x13 << SYSCON_LDOSPCAL_NOPLL_SHIFT) /* 0.95 V */
#  define SYSCON_LDOSPCAL_NOPLL_1p00V   (0x14 << SYSCON_LDOSPCAL_NOPLL_SHIFT) /* 1.00 V */
#  define SYSCON_LDOSPCAL_NOPLL_1p05V   (0x15 << SYSCON_LDOSPCAL_NOPLL_SHIFT) /* 1.05 V */
#  define SYSCON_LDOSPCAL_NOPLL_1p10V   (0x16 << SYSCON_LDOSPCAL_NOPLL_SHIFT) /* 1.10 V */
#  define SYSCON_LDOSPCAL_NOPLL_1p15V   (0x17 << SYSCON_LDOSPCAL_NOPLL_SHIFT) /* 1.15 V */
#  define SYSCON_LDOSPCAL_NOPLL_1p20V   (0x18 << SYSCON_LDOSPCAL_NOPLL_SHIFT) /* 1.20 V */

#define SYSCON_LDOSPCAL_WITHPLL_SHIFT   (8)     /* Bits 15-8: Sleep with PLL */
#define SYSCON_LDOSPCAL_WITHPLL_MASK    (0xff << SYSCON_LDOSPCAL_WITHPLL_SHIFT)
#  define SYSCON_LDOSPCAL_WITHPLL_0p90V (0x12 << SYSCON_LDOSPCAL_WITHPLL_SHIFT) /* 0.90 V */
#  define SYSCON_LDOSPCAL_WITHPLL_0p95V (0x13 << SYSCON_LDOSPCAL_WITHPLL_SHIFT) /* 0.95 V */
#  define SYSCON_LDOSPCAL_WITHPLL_1p00V (0x14 << SYSCON_LDOSPCAL_WITHPLL_SHIFT) /* 1.00 V */
#  define SYSCON_LDOSPCAL_WITHPLL_1p05V (0x15 << SYSCON_LDOSPCAL_WITHPLL_SHIFT) /* 1.05 V */
#  define SYSCON_LDOSPCAL_WITHPLL_1p10V (0x16 << SYSCON_LDOSPCAL_WITHPLL_SHIFT) /* 1.10 V */
#  define SYSCON_LDOSPCAL_WITHPLL_1p15V (0x17 << SYSCON_LDOSPCAL_WITHPLL_SHIFT) /* 1.15 V */
#  define SYSCON_LDOSPCAL_WITHPLL_1p20V (0x18 << SYSCON_LDOSPCAL_WITHPLL_SHIFT) /* 1.20 V */

/* LDO Deep-Sleep Power Control */

#define SYSCON_LDODPCTL_VLDO_SHIFT     (0)       /* Bits 7-0: LDO Output Voltage */
#define SYSCON_LDODPCTL_VLDO_MASK      (0xff << SYSCON_LDODPCTL_VLDO_SHIFT)
#  define SYSCON_LDODPCTL_VLDO_0p90V   (0x12 << SYSCON_LDODPCTL_VLDO_SHIFT) /* 0.90 V */
#  define SYSCON_LDODPCTL_VLDO_0p95V   (0x13 << SYSCON_LDODPCTL_VLDO_SHIFT) /* 0.95 V */
#  define SYSCON_LDODPCTL_VLDO_1p00V   (0x14 << SYSCON_LDODPCTL_VLDO_SHIFT) /* 1.00 V */
#  define SYSCON_LDODPCTL_VLDO_1p05V   (0x15 << SYSCON_LDODPCTL_VLDO_SHIFT) /* 1.05 V */
#  define SYSCON_LDODPCTL_VLDO_1p10V   (0x16 << SYSCON_LDODPCTL_VLDO_SHIFT) /* 1.10 V */
#  define SYSCON_LDODPCTL_VLDO_1p15V   (0x17 << SYSCON_LDODPCTL_VLDO_SHIFT) /* 1.15 V */
#  define SYSCON_LDODPCTL_VLDO_1p20V   (0x18 << SYSCON_LDODPCTL_VLDO_SHIFT) /* 1.20 V */

#define SYSCON_LDODPCTL_VADJEN         (1 << 31) /* Bit 31: Voltage Adjust Enable */

/* LDO Deep-Sleep Power Calibration */

#define SYSCON_LDODPCAL_NOPLL_SHIFT    (0)       /* Bits 7-0: Deep-Sleep without PLL */
#define SYSCON_LDODPCAL_NOPLL_MASK     (0xff << SYSCON_LDODPCAL_NOPLL_SHIFT)
#  define SYSCON_LDODPCAL_NOPLL_0p90V  (0x12 << SYSCON_LDODPCAL_NOPLL_SHIFT) /* 0.90 V */
#  define SYSCON_LDODPCAL_NOPLL_0p95V  (0x13 << SYSCON_LDODPCAL_NOPLL_SHIFT) /* 0.95 V */
#  define SYSCON_LDODPCAL_NOPLL_1p00V  (0x14 << SYSCON_LDODPCAL_NOPLL_SHIFT) /* 1.00 V */
#  define SYSCON_LDODPCAL_NOPLL_1p05V  (0x15 << SYSCON_LDODPCAL_NOPLL_SHIFT) /* 1.05 V */
#  define SYSCON_LDODPCAL_NOPLL_1p10V  (0x16 << SYSCON_LDODPCAL_NOPLL_SHIFT) /* 1.10 V */
#  define SYSCON_LDODPCAL_NOPLL_1p15V  (0x17 << SYSCON_LDODPCAL_NOPLL_SHIFT) /* 1.15 V */
#  define SYSCON_LDODPCAL_NOPLL_1p20V  (0x18 << SYSCON_LDODPCAL_NOPLL_SHIFT) /* 1.20 V */

#define SYSCON_LDODPCAL_30KHZ_SHIFT    (8)       /* Bits 15-8: Deep-Sleep with IOSC */
#define SYSCON_LDODPCAL_30KHZ_MASK     (0xff << SYSCON_LDODPCAL_30KHZ_SHIFT)
#  define SYSCON_LDODPCAL_30KHZ_0p90V  (0x12 << SYSCON_LDODPCAL_30KHZ_SHIFT) /* 0.90 V */
#  define SYSCON_LDODPCAL_30KHZ_0p95V  (0x13 << SYSCON_LDODPCAL_30KHZ_SHIFT) /* 0.95 V */
#  define SYSCON_LDODPCAL_30KHZ_1p00V  (0x14 << SYSCON_LDODPCAL_30KHZ_SHIFT) /* 1.00 V */
#  define SYSCON_LDODPCAL_30KHZ_1p05V  (0x15 << SYSCON_LDODPCAL_30KHZ_SHIFT) /* 1.05 V */
#  define SYSCON_LDODPCAL_30KHZ_1p10V  (0x16 << SYSCON_LDODPCAL_30KHZ_SHIFT) /* 1.10 V */
#  define SYSCON_LDODPCAL_30KHZ_1p15V  (0x17 << SYSCON_LDODPCAL_30KHZ_SHIFT) /* 1.15 V */
#  define SYSCON_LDODPCAL_30KHZ_1p20V  (0x18 << SYSCON_LDODPCAL_30KHZ_SHIFT) /* 1.20 V */

/* Sleep / Deep-Sleep Power Mode Status */

#define SYSCON_SDPMST_SPDERR          (1 << 0)  /* Bit 0:  SRAM Power Down Request Error */
#define SYSCON_SDPMST_FPDERR          (1 << 1)  /* Bit 1:  Flash Memory Power Down Request Error */
#define SYSCON_SDPMST_PPDERR          (1 << 2)  /* Bit 2:  PIOSC Power Down Request Error */
#define SYSCON_SDPMST_LDMINERR        (1 << 3)  /* Bit 3:  VLDO Value Below Minimum Error in Deep-Sleep Mode */
#define SYSCON_SDPMST_LSMINERR        (1 << 4)  /* Bit 4:  VLDO Value Below Minimum Error in Sleep Mode */
#define SYSCON_SDPMST_LMAXERR         (1 << 6)  /* Bit 6:  VLDO Value Above Maximum Error */
#define SYSCON_SDPMST_PPDW            (1 << 7)  /* Bit 7:  PIOSC Power Down Request Warning */
#define SYSCON_SDPMST_PRACT           (1 << 16) /* Bit 16: Sleep or Deep-Sleep Power Request Active */
#define SYSCON_SDPMST_LOWPWR          (1 << 17) /* Bit 17: Sleep or Deep-Sleep Mode */
#define SYSCON_SDPMST_FLASHLP         (1 << 18) /* Bit 18: Flash Memory in Low Power State */
#define SYSCON_SDPMST_LDOUA           (1 << 19) /* Bit 19: LDO Update Active */

/* Reset Behavior Control Register */

#define SYSCON_RESBEHAVCTL_EXTRES_SHIFT    (0)  /* Bits 0-1: External RST Pin Operation */
#define SYSCON_RESBEHAVCTL_EXTRES_MASK     (3 << SYSCON_RESBEHAVCTL_EXTRES_SHIFT)
#  define SYSCON_RESBEHAVCTL_EXTRES_SYSRST (2 << SYSCON_RESBEHAVCTL_EXTRES_SHIFT) /* System reset */
#  define SYSCON_RESBEHAVCTL_EXTRES_POR    (3 << SYSCON_RESBEHAVCTL_EXTRES_SHIFT) /* POR */

#define SYSCON_RESBEHAVCTL_BOR_SHIFT       (2)  /* Bits 2-3: BOR Reset operation */
#define SYSCON_RESBEHAVCTL_BOR_MASK        (3 << SYSCON_RESBEHAVCTL_BOR_SHIFT)
#  define SYSCON_RESBEHAVCTL_BOR_SYSRST    (2 << SYSCON_RESBEHAVCTL_BOR_SHIFT) /* System reset */
#  define SYSCON_RESBEHAVCTL_BOR_POR       (3 << SYSCON_RESBEHAVCTL_BOR_SHIFT) /* POR */

#define SYSCON_RESBEHAVCTL_WDOG0_SHIFT     (4)  /* Bits 4-5: Watchdog 0 Reset Operation */
#define SYSCON_RESBEHAVCTL_WDOG0_MASK      (3 << SYSCON_RESBEHAVCTL_WDOG0_SHIFT)
#  define SYSCON_RESBEHAVCTL_WDOG0_SYSRST  (2 << SYSCON_RESBEHAVCTL_WDOG0_SHIFT) /* System reset */
#  define SYSCON_RESBEHAVCTL_WDOG0_POR     (3 << SYSCON_RESBEHAVCTL_WDOG0_SHIFT) /* POR */

#define SYSCON_RESBEHAVCTL_WDOG1_SHIFT     (6)  /* Bits 6-7: Watchdog 1 Reset Operation */
#define SYSCON_RESBEHAVCTL_WDOG1_MASK      (3 << SYSCON_RESBEHAVCTL_WDOG1_SHIFT)
#  define SYSCON_RESBEHAVCTL_WDOG1_SYSRST  (2 << SYSCON_RESBEHAVCTL_WDOG1_SHIFT) /* System reset */
#  define SYSCON_RESBEHAVCTL_WDOG1_POR     (3 << SYSCON_RESBEHAVCTL_WDOG1_SHIFT) /* POR */

/* Hardware System Service Request */

#define SYSCON_HSSR_CDOFF_SHIFT        (0)      /* Bits 0-23: Command Descriptor Pointer */
#define SYSCON_HSSR_CDOFF_MASK         (0xffffff << SYSCON_HSSR_CDOFF_SHIFT)
#  define SYSCON_HSSR_CDOFF(n)         ((uint32_t)(n) << SYSCON_HSSR_CDOFF_SHIFT)
#  define SYSCON_HSSR_CDOFF_NOREQ      (0 << SYSCON_HSSR_CDOFF_SHIFT)        /* No request pending */
#  define SYSCON_HSSR_CDOFF_ERROR      (0xffffff << SYSCON_HSSR_CDOFF_SHIFT) /* An error occurred */

#define SYSCON_HSSR_KEY_SHIFT          (24)     /* Bit 24-31: Write Key */
#define SYSCON_HSSR_KEY_MASK           (0xff << SYSCON_HSSR_KEY_SHIFT)
#  define SYSCON_HSSR_KEY              (0xca << SYSCON_HSSR_KEY_SHIFT) /* Key value */

/* USB Power Domain Status */

#define SYSCON_USBPDS_PWRSTAT_SHIFT    (0)      /* Bits 0-1: Power Domain Status */
#define SYSCON_USBPDS_PWRSTAT_MASK     (3 << SYSCON_USBPDS_PWRSTAT_SHIFT)
#  define SYSCON_USBPDS_PWRSTAT_OFF    (0 << SYSCON_USBPDS_PWRSTAT_SHIFT) /* OFF */
#  define SYSCON_USBPDS_PWRSTAT_ON     (3 << SYSCON_USBPDS_PWRSTAT_SHIFT) /* ON */

#define SYSCON_USBPDS_MEMSTAT_SHIFT    (2)      /* Bits 2-3: Memory Array Power Status */
#define SYSCON_USBPDS_MEMSTAT_MASK     (3 << SYSCON_USBPDS_PWRSTAT_SHIFT)
#  define SYSCON_USBPDS_MEMSTAT_OFF    (0 << SYSCON_USBPDS_PWRSTAT_SHIFT) /* Array OFF */
#  define SYSCON_USBPDS_MEMSTAT_RETAIN (1 << SYSCON_USBPDS_PWRSTAT_SHIFT) /* SRAM Retention */
#  define SYSCON_USBPDS_MEMSTAT_ON     (3 << SYSCON_USBPDS_PWRSTAT_SHIFT) /* Array On */

/* USB Memory Power Control */

#define SYSCON_USBMPC_PWRCTL_SHIFT     (0)      /* Bits 0-1: Memory Array Power Control */
#define SYSCON_USBMPC_PWRCTL_MASK      (3 << SYSCON_USBMPC_PWRCTL_SHIFT)
#  define SYSCON_USBMPC_PWRCTL_OFF     (0 << SYSCON_USBMPC_PWRCTL_SHIFT) /* Array OFF */
#  define SYSCON_USBMPC_PWRCTL_RETAIN  (1 << SYSCON_USBMPC_PWRCTL_SHIFT) /* SRAM Retention */
#  define SYSCON_USBMPC_PWRCTL_ON      (3 << SYSCON_USBMPC_PWRCTL_SHIFT) /* Array On */

/* Ethernet MAC Power Domain Status */

#define SYSCON_EMACPDS_PWRSTAT_SHIFT   (0)      /* Bits 0-1: Power Domain Status */
#define SYSCON_EMACPDS_PWRSTAT_MASK    (3 << SYSCON_EMACPDS_PWRSTAT_SHIFT)
#  define SYSCON_EMACPDS_PWRSTAT_OFF   (0 << SYSCON_EMACPDS_PWRSTAT_SHIFT) /* OFF */
#  define SYSCON_EMACPDS_PWRSTAT_ON    (3 << SYSCON_EMACPDS_PWRSTAT_SHIFT) /* ON */

#define SYSCON_EMACPDS_MEMSTAT_SHIFT   (2)      /* Bits 2-3: Memory Array Power Status */
#define SYSCON_EMACPDS_MEMSTAT_MASK    (3 << SYSCON_EMACPDS_MEMSTAT_SHIFT)
#  define SYSCON_EMACPDS_MEMSTAT_OFF   (0 << SYSCON_EMACPDS_MEMSTAT_SHIFT) /* Array OFF */
#  define SYSCON_EMACPDS_MEMSTAT_ON    (3 << SYSCON_EMACPDS_MEMSTAT_SHIFT) /* Array On */

/* Ethernet MAC Memory Power Control */

#define SYSCON_EMACMPC_PWRCTL_SHIFT    (0)      /* Bits 0-1: Memory Array Power Control */
#define SYSCON_EMACMPC_PWRCTL_MASK     (3 << SYSCON_EMACMPC_PWRCTL_SHIFT)
#define SYSCON_EMACMPC_PWRCTL_OFF      (0 << SYSCON_EMACMPC_PWRCTL_SHIFT) /* Array OFF */
#define SYSCON_EMACMPC_PWRCTL_ON       (3 << SYSCON_EMACMPC_PWRCTL_SHIFT) /* Array On */

/* LCD Power Domain Status */

#define SYSCON_LCDPDS_PWRSTAT_SHIFT    (0)      /* Bits 0-1: Power Domain Status */
#define SYSCON_LCDPDS_PWRSTAT_MASK     (3 << SYSCON_LCDPDS_PWRSTAT_SHIFT)
#  define SYSCON_LCDPDS_PWRSTAT_OFF    (0 << SYSCON_LCDPDS_PWRSTAT_SHIFT) /* OFF */
#  define SYSCON_LCDPDS_PWRSTAT_ON     (3 << SYSCON_LCDPDS_PWRSTAT_SHIFT) /* ON */

#define SYSCON_LCDPDS_MEMSTAT_SHIFT    (2)      /* Bits 2-3: Memory Array Power Status */
#define SYSCON_LCDPDS_MEMSTAT_MASK     (3 << SYSCON_LCDPDS_MEMSTAT_SHIFT)
#  define SYSCON_LCDPDS_MEMSTAT_OFF    (0 << SYSCON_LCDPDS_MEMSTAT_SHIFT) /* Array OFF */
#  define SYSCON_LCDPDS_MEMSTAT_ON     (3 << SYSCON_LCDPDS_MEMSTAT_SHIFT) /* Array On */

/* LCD Memory Power Control */

#define SYSCON_LCDMPC_PWRCTL_SHIFT     (0)      /* Bits 0-1: Memory Array Power Control */
#define SYSCON_LCDMPC_PWRCTL_MASK      (3 << SYSCON_LCDMPC_PWRCTL_SHIFT)
#define SYSCON_LCDMPC_PWRCTL_OFF       (0 << SYSCON_LCDMPC_PWRCTL_SHIFT) /* Array OFF */
#define SYSCON_LCDMPC_PWRCTL_ON        (3 << SYSCON_LCDMPC_PWRCTL_SHIFT) /* Array On */

/* CAN 0 Power Domain Status */

#define SYSCON_CAN0PDS_PWRSTAT_SHIFT   (0)      /* Bits 0-1: Power Domain Status */
#define SYSCON_CAN0PDS_PWRSTAT_MASK    (3 << SYSCON_CAN0PDS_PWRSTAT_SHIFT)
#  define SYSCON_CAN0PDS_PWRSTAT_OFF   (0 << SYSCON_CAN0PDS_PWRSTAT_SHIFT) /* OFF */
#  define SYSCON_CAN0PDS_PWRSTAT_ON    (3 << SYSCON_CAN0PDS_PWRSTAT_SHIFT) /* ON */

#define SYSCON_CAN0PDS_MEMSTAT_SHIFT   (2)      /* Bits 2-3: Memory Array Power Status */
#define SYSCON_CAN0PDS_MEMSTAT_MASK    (3 << SYSCON_CAN0PDS_MEMSTAT_SHIFT)
#  define SYSCON_CAN0PDS_MEMSTAT_OFF   (0 << SYSCON_CAN0PDS_MEMSTAT_SHIFT) /* Array OFF */
#  define SYSCON_CAN0PDS_MEMSTAT_ON    (3 << SYSCON_CAN0PDS_MEMSTAT_SHIFT) /* Array On */

/* CAN 0 Memory Power Control */

#define SYSCON_CAN0MPC_PWRCTL_SHIFT    (0)      /* Bits 0-1: Memory Array Power Control */
#define SYSCON_CAN0MPC_PWRCTL_MASK     (3 << SYSCON_CAN0MPC_PWRCTL_SHIFT)
#define SYSCON_CAN0MPC_PWRCTL_OFF      (0 << SYSCON_CAN0MPC_PWRCTL_SHIFT) /* Array OFF */
#define SYSCON_CAN0MPC_PWRCTL_ON       (3 << SYSCON_CAN0MPC_PWRCTL_SHIFT) /* Array On */

/* CAN 1 Power Domain Status */

#define SYSCON_CAN1PDS_PWRSTAT_SHIFT   (0)      /* Bits 0-1: Power Domain Status */
#define SYSCON_CAN1PDS_PWRSTAT_MASK    (3 << SYSCON_CAN1PDS_PWRSTAT_SHIFT)
#  define SYSCON_CAN1PDS_PWRSTAT_OFF   (0 << SYSCON_CAN1PDS_PWRSTAT_SHIFT) /* OFF */
#  define SYSCON_CAN1PDS_PWRSTAT_ON    (3 << SYSCON_CAN1PDS_PWRSTAT_SHIFT) /* ON */

#define SYSCON_CAN1PDS_MEMSTAT_SHIFT   (2)      /* Bits 2-3: Memory Array Power Status */
#define SYSCON_CAN1PDS_MEMSTAT_MASK    (3 << SYSCON_CAN1PDS_MEMSTAT_SHIFT)
#  define SYSCON_CAN1PDS_MEMSTAT_OFF   (0 << SYSCON_CAN1PDS_MEMSTAT_SHIFT) /* Array OFF */
#  define SYSCON_CAN1PDS_MEMSTAT_ON    (3 << SYSCON_CAN1PDS_MEMSTAT_SHIFT) /* Array On */

/* CAN 1 Memory Power Control */

#define SYSCON_CAN1MPC_PWRCTL_SHIFT    (0)      /* Bits 0-1: Memory Array Power Control */
#define SYSCON_CAN1MPC_PWRCTL_MASK     (3 << SYSCON_CAN1MPC_PWRCTL_SHIFT)
#define SYSCON_CAN1MPC_PWRCTL_OFF      (0 << SYSCON_CAN1MPC_PWRCTL_SHIFT) /* Array OFF */
#define SYSCON_CAN1MPC_PWRCTL_ON       (3 << SYSCON_CAN1MPC_PWRCTL_SHIFT) /* Array On */

/* Watchdog Timer Peripheral Present */

#define SYSCON_PPWD(n)                 (1 << (n)) /* Bit n:  WDTn present */
#  define SYSCON_PPWD_P0               (1 << 0)   /* Bit 0:  WDT0 present */
#  define SYSCON_PPWD_P1               (1 << 1)   /* Bit 1:  WDT1 present */

/* 16/32-Bit Timer Peripheral Present */

#define SYSCON_PPTIMER(n)              (1 << (n)) /* Bit n: 16/32-Bit Timer n Present */
#  define SYSCON_PPTIMER_P0            (1 << 0)   /* Bit 0: 16/32-Bit Timer 0 Present */
#  define SYSCON_PPTIMER_P1            (1 << 1)   /* Bit 1: 16/32-Bit Timer 0 Present */
#  define SYSCON_PPTIMER_P2            (1 << 2)   /* Bit 2: 16/32-Bit Timer 0 Present */
#  define SYSCON_PPTIMER_P3            (1 << 3)   /* Bit 3: 16/32-Bit Timer 0 Present */
#  define SYSCON_PPTIMER_P4            (1 << 4)   /* Bit 4: 16/32-Bit Timer 0 Present */
#  define SYSCON_PPTIMER_P5            (1 << 5)   /* Bit 5: 16/32-Bit Timer 0 Present */
#  define SYSCON_PPTIMER_P6            (1 << 6)   /* Bit 6: 16/32-Bit Timer 0 Present */
#  define SYSCON_PPTIMER_P7            (1 << 7)   /* Bit 7: 16/32-Bit Timer 0 Present */

/* GPIO Peripheral Present */

#define SYSCON_PPGPIO(n)               (1 << (n)) /* Bit n:  GPIO Port n Present */
#  define SYSCON_PPGPIO_P0             (1 << 0)   /* Bit 0:  GPIO Port A Present */
#  define SYSCON_PPGPIO_P1             (1 << 1)   /* Bit 1:  GPIO Port B Present */
#  define SYSCON_PPGPIO_P2             (1 << 2)   /* Bit 2:  GPIO Port C Present */
#  define SYSCON_PPGPIO_P3             (1 << 3)   /* Bit 3:  GPIO Port D Present */
#  define SYSCON_PPGPIO_P4             (1 << 4)   /* Bit 4:  GPIO Port E Present */
#  define SYSCON_PPGPIO_P5             (1 << 5)   /* Bit 5:  GPIO Port F Present */
#  define SYSCON_PPGPIO_P6             (1 << 6)   /* Bit 6:  GPIO Port G Present */
#  define SYSCON_PPGPIO_P7             (1 << 7)   /* Bit 7:  GPIO Port H Present */
#  define SYSCON_PPGPIO_P8             (1 << 8)   /* Bit 8:  GPIO Port J Present */
#  define SYSCON_PPGPIO_P9             (1 << 9)   /* Bit 9:  GPIO Port K Present */
#  define SYSCON_PPGPIO_P10            (1 << 10)  /* Bit 10: GPIO Port L Present */
#  define SYSCON_PPGPIO_P11            (1 << 11)  /* Bit 11: GPIO Port M Present */
#  define SYSCON_PPGPIO_P12            (1 << 12)  /* Bit 12: GPIO Port N Present */
#  define SYSCON_PPGPIO_P13            (1 << 13)  /* Bit 13: GPIO Port P Present */
#  define SYSCON_PPGPIO_P14            (1 << 14)  /* Bit 14: GPIO Port Q Present */
#  define SYSCON_PPGPIO_P15            (1 << 15)  /* Bit 15: GPIO Port R Present */
#  define SYSCON_PPGPIO_P16            (1 << 16)  /* Bit 16: GPIO Port S Present */
#  define SYSCON_PPGPIO_P17            (1 << 17)  /* Bit 17: GPIO Port T Present */

/* μDMA Peripheral Present */

#define SYSCON_PPDMA_P0                (1 << 0)   /* Bit 0:  μDMA Module Present */

/* EPI Peripheral Present */

#define SYSCON_PPEPI_P0                (1 << 0)   /* Bit 0: EPI Module Present */

/* Hibernation Peripheral Present */

#define SYSCON_PPHIB_P0                (1 << 0)   /* Bit 0: Hibernation Module Present */

/* UART Peripheral Present */

#define SYSCON_PPUART(n)               (1 << (n)) /* Bit n:  UART Module n Present */
#  define SYSCON_PPUART_P0             (1 << 0)   /* Bit 0:  UART Module 0 Present */
#  define SYSCON_PPUART_P1             (1 << 1)   /* Bit 1:  UART Module 1 Present */
#  define SYSCON_PPUART_P2             (1 << 2)   /* Bit 2:  UART Module 2 Present */
#  define SYSCON_PPUART_P3             (1 << 3)   /* Bit 3:  UART Module 3 Present */
#  define SYSCON_PPUART_P4             (1 << 4)   /* Bit 4:  UART Module 4 Present */
#  define SYSCON_PPUART_P5             (1 << 5)   /* Bit 5:  UART Module 5 Present */
#  define SYSCON_PPUART_P6             (1 << 6)   /* Bit 6:  UART Module 6 Present */
#  define SYSCON_PPUART_P7             (1 << 7)   /* Bit 7:  UART Module 7 Present */

/* SSI Peripheral Present */

#define SYSCON_PPSSI(n)                (1 << (n)) /* Bit n:  SSI Module n Present */
#  define SYSCON_PPSSI_P0              (1 << 0)   /* Bit 0:  SSI Module 0 Present */
#  define SYSCON_PPSSI_P1              (1 << 1)   /* Bit 1:  SSI Module 1 Present */
#  define SYSCON_PPSSI_P2              (1 << 2)   /* Bit 2:  SSI Module 2 Present */
#  define SYSCON_PPSSI_P3              (1 << 3)   /* Bit 3:  SSI Module 3 Present */

/* I2C Peripheral Present */

#define SYSCON_PPI2C(n)                (1 << (n)) /* Bit n:  I2C Module n Present */
#  define SYSCON_PPI2C_P0              (1 << 0)   /* Bit 0:  I2C Module 0 Present */
#  define SYSCON_PPI2C_P1              (1 << 1)   /* Bit 1:  I2C Module 1 Present */
#  define SYSCON_PPI2C_P2              (1 << 2)   /* Bit 2:  I2C Module 2 Present */
#  define SYSCON_PPI2C_P3              (1 << 3)   /* Bit 3:  I2C Module 3 Present */
#  define SYSCON_PPI2C_P4              (1 << 4)   /* Bit 4:  I2C Module 4 Present */
#  define SYSCON_PPI2C_P5              (1 << 5)   /* Bit 5:  I2C Module 5 Present */
#  define SYSCON_PPI2C_P6              (1 << 6)   /* Bit 6:  I2C Module 6 Present */
#  define SYSCON_PPI2C_P7              (1 << 7)   /* Bit 7:  I2C Module 7 Present */
#  define SYSCON_PPI2C_P8              (1 << 8)   /* Bit 8:  I2C Module 8 Present */
#  define SYSCON_PPI2C_P9              (1 << 9)   /* Bit 9:  I2C Module 9 Present */

/* USB Peripheral Present */

#define SYSCON_PPUSB_P0                (1 << 0)   /* Bit 0: USB Module Present */

/* Ethernet PHY Peripheral Present */

#define SYSCON_PPEPHY_P0               (1 << 0)   /* Bit 0: Ethernet PHY Module Present */

/* CAN Peripheral Present */

#define SYSCON_PPCAN(n)                (1 << (n)) /* Bit n:  CAN Module n Present */
#  define SYSCON_PPCAN_P0              (1 << 0)   /* Bit 0:  CAN Module 0 Present */
#  define SYSCON_PPCAN_P1              (1 << 1)   /* Bit 1:  CAN Module 1 Present */

/* ADC Peripheral Present */

#define SYSCON_PPADC(n)                (1 << (n)) /* Bit n:  ADC Module n Present */
#  define SYSCON_PPADC_P0              (1 << 0)   /* Bit 0:  ADC Module 0 Present */
#  define SYSCON_PPADC_P1              (1 << 1)   /* Bit 1:  ADC Module 1 Present */

/* ACMP Peripheral Present */

#define SYSCON_PPACMP_P0               (1 << 0)   /* Bit 0:  Analog Comparator Module Present */

/* PWM Peripheral Present */

#define SYSCON_PPWM(n)                 (1 << (n)) /* Bit n:  PWM Module n Present */
#  define SYSCON_PPWM_P0               (1 << 0)   /* Bit 0:  PWM Module 0 Present */
#  define SYSCON_PPWM_P1               (1 << 1)   /* Bit 1:  PWM Module 1 Present */

/* QE Interface Peripheral Present */

#define SYSCON_PPQEI(n)                (1 << (n)) /* Bit n:  QEI Module n Present */
#  define SYSCON_PPQEI_P0              (1 << 0)   /* Bit 0:  QEI Module 0 Present */
#  define SYSCON_PPUART_P1             (1 << 1)   /* Bit 1:  QEI Module 1 Present */

/* Low Pin Count Interface Peripheral Present */

#define SYSCON_PPLPC_P0                (1 << 0)   /* Bit 0: LPC Module Present */

/* Platform Environment Control Interface Peripheral Present */

#define SYSCON_PPPECI_P0               (1 << 0)   /* Bit 0: PECI Module Present */

/* Fan Control Peripheral Present */

#define SYSCON_PPFAN_P0                (1 << 0)   /* Bit 0: FAN Module 0 Present */

/* EEPROM Peripheral Present */

#define SYSCON_PPEEPROM_P0             (1 << 0)   /* Bit 0:  EEPROM Module Present */

/* 32/64-Bit Wide Timer Peripheral Present */

#define SYSCON_PPWTIMER(n)             (1 << (n)) /* Bit n:  32/64-Bit Wide Timer n Present */
#  define SYSCON_PPWTIMER_P0           (1 << 0)   /* Bit 0:  32/64-Bit Wide Timer 0 Present */
#  define SYSCON_PPWTIMER_P1           (1 << 1)   /* Bit 1:  32/64-Bit Wide Timer 1 Present */
#  define SYSCON_PPWTIMER_P2           (1 << 2)   /* Bit 2:  32/64-Bit Wide Timer 2 Present */
#  define SYSCON_PPWTIMER_P3           (1 << 3)   /* Bit 3:  32/64-Bit Wide Timer 3 Present */
#  define SYSCON_PPWTIMER_P4           (1 << 4)   /* Bit 4:  32/64-Bit Wide Timer 4 Present */
#  define SYSCON_PPWTIMER_P5           (1 << 5)   /* Bit 5:  32/64-Bit Wide Timer 5 Present */

/* Remote Temperature Sensor Peripheral Present */

#define SYSCON_PPRTS_P0                (1 << 0)   /* Bit 0: RTS Module Present */

/* CRC/Crypto Modules Peripheral Present */

#define SYSCON_PPCCM_P0                (1 << 0)   /* Bit 0: CRC/Crypto Modules Present */

/* LCD Peripheral Present */

#define SYSCON_PPLCD_P0                (1 << 0)   /* Bit 0: LCD Module Present */

/* 1-Wire Peripheral Present */

#define SYSCON_PPOWIRE_P0              (1 << 0)   /* Bit 0: 1-Wire Module Present */

/* Ethernet MAC Peripheral Present */

#define SYSCON_PPEMAC_P0               (1 << 0)   /* Bit 0: Ethernet Controller Module Present */

/* Power Regulator Bus Peripheral Present */

#define SYSCON_PPPRB__P0               (1 << 0)   /* Bit 0: PRB Module Present */

/* Human Interface Master Peripheral Present */

#define SYSCON_PPHIM_P0                (1 << 0)   /* Bit 0: HIM Module Present */

/* Watchdog Timer Software Reset */

#define SYSCON_SRWD(n)                 (1 << (n)) /* Bit n:  Watchdog Timer n Software Reset */
#  define SYSCON_SRWD_R0               (1 << 0)   /* Bit 0:  Watchdog Timer 0 Software Reset */
#  define SYSCON_SRWD_R1               (1 << 1)   /* Bit 1:  Watchdog Timer 1 Software Reset */

/* 16/32-Bit Timer Software Reset */

#define SYSCON_SRTIMER(n)              (1 << (n)) /* Bit n:  16/32-Bit Timer n Software Reset */
#  define SYSCON_SRTIMER_R0            (1 << 0)   /* Bit 0:  16/32-Bit Timer 0 Software Reset */
#  define SYSCON_SRTIMER_R1            (1 << 1)   /* Bit 1:  16/32-Bit Timer 1 Software Reset */
#  define SYSCON_SRTIMER_R2            (1 << 2)   /* Bit 2:  16/32-Bit Timer 2 Software Reset */
#  define SYSCON_SRTIMER_R3            (1 << 3)   /* Bit 3:  16/32-Bit Timer 3 Software Reset */
#  define SYSCON_SRTIMER_R4            (1 << 4)   /* Bit 4:  16/32-Bit Timer 4 Software Reset */
#  define SYSCON_SRTIMER_R5            (1 << 5)   /* Bit 5:  16/32-Bit Timer 5 Software Reset */
#  define SYSCON_SRTIMER_R6            (1 << 6)   /* Bit 6:  16/32-Bit Timer 6 Software Reset */
#  define SYSCON_SRTIMER_R7            (1 << 7)   /* Bit 7:  16/32-Bit Timer 7 Software Reset */

/* GPIO Software Reset */

#define SYSCON_SRGPIO(n)               (1 << (n)) /* Bit n:  GPIO Port n Software Reset */
#  define SYSCON_SRGPIO_R0             (1 << 0)   /* Bit 0:  GPIO Port A Software Reset */
#  define SYSCON_SRGPIO_R1             (1 << 1)   /* Bit 1:  GPIO Port B Software Reset */
#  define SYSCON_SRGPIO_R2             (1 << 2)   /* Bit 2:  GPIO Port C Software Reset */
#  define SYSCON_SRGPIO_R3             (1 << 3)   /* Bit 3:  GPIO Port D Software Reset */
#  define SYSCON_SRGPIO_R4             (1 << 4)   /* Bit 4:  GPIO Port E Software Reset */
#  define SYSCON_SRPGIO_R5             (1 << 5)   /* Bit 5:  GPIO Port F Software Reset */
#  define SYSCON_SRPGIO_R6             (1 << 6)   /* Bit 6:  GPIO Port G Software Reset */
#  define SYSCON_SRPGIO_R7             (1 << 7)   /* Bit 7:  GPIO Port H Software Reset */
#  define SYSCON_SRPGIO_R8             (1 << 8)   /* Bit 8:  GPIO Port J Software Reset */
#  define SYSCON_SRPGIO_R9             (1 << 9)   /* Bit 9:  GPIO Port K Software Reset */
#  define SYSCON_SRPGIO_R10            (1 << 10)  /* Bit 10: GPIO Port L Software Reset */
#  define SYSCON_SRPGIO_R11            (1 << 11)  /* Bit 11: GPIO Port M Software Reset */
#  define SYSCON_SRPGIO_R12            (1 << 12)  /* Bit 12: GPIO Port N Software Reset */
#  define SYSCON_SRPGIO_R13            (1 << 13)  /* Bit 13: GPIO Port P Software Reset */
#  define SYSCON_SRPGIO_R14            (1 << 14)  /* Bit 14: GPIO Port Q Software Reset */
#  define SYSCON_SRPGIO_R15            (1 << 15)  /* Bit 15: GPIO Port R Software Reset */
#  define SYSCON_SRPGIO_R16            (1 << 16)  /* Bit 16: GPIO Port S Software Reset */
#  define SYSCON_SRPGIO_R17            (1 << 17)  /* Bit 17: GPIO Port T Software Reset */

/* μDMA Software Reset */

#define SYSCON_SRDMA_R0                (1 << 0)   /* Bit 0:  μDMA Module Software Reset */

/* EPI Software Reset */

#define SYSCON_SREPI_R0                (1 << 0)   /* Bit 0: EPI Module Software Reset */

/* Hibernation Software Reset */

#define SYSCON_SRHIB_R0                (1 << 0)   /* Bit 0:  Hibernation Module Software Reset */

/* UART Software Reset */

#define SYSCON_SRUARTR(n)              (1 << (n)) /* Bit n:  UART Module n Software Reset */
#  define SYSCON_SRUARTR_R0            (1 << 0)   /* Bit 0:  UART Module 0 Software Reset */
#  define SYSCON_SRUARTR_R1            (1 << 1)   /* Bit 1:  UART Module 1 Software Reset */
#  define SYSCON_SRUARTR_R2            (1 << 2)   /* Bit 2:  UART Module 2 Software Reset */
#  define SYSCON_SRUARTR_R3            (1 << 3)   /* Bit 3:  UART Module 3 Software Reset */
#  define SYSCON_SRUARTR_R4            (1 << 4)   /* Bit 4:  UART Module 4 Software Reset */
#  define SYSCON_SRUARTR_R5            (1 << 5)   /* Bit 5:  UART Module 5 Software Reset */
#  define SYSCON_SRUARTR_R6            (1 << 6)   /* Bit 6:  UART Module 6 Software Reset */
#  define SYSCON_SRUARTR_R7            (1 << 7)   /* Bit 7:  UART Module 7 Software Reset */

/* SSI Software Reset */

#define SYSCON_SRSSI(n)                (1 << (n)) /* Bit n:  SSI Module n Software Reset */
#  define SYSCON_SRSSI_R0              (1 << 0)   /* Bit 0:  SSI Module 0 Software Reset */
#  define SYSCON_SRSSI_R1              (1 << 1)   /* Bit 1:  SSI Module 1 Software Reset */
#  define SYSCON_SRSSI_R2              (1 << 2)   /* Bit 2:  SSI Module 2 Software Reset */
#  define SYSCON_SRSSI_R3              (1 << 3)   /* Bit 3:  SSI Module 3 Software Reset */

/* I2C Software Reset */

#define SYSCON_SRI2C(n)                (1 << (n)) /* Bit n:  I2C Module n Software Reset */
#  define SYSCON_SRI2C_R0              (1 << 0)   /* Bit 0:  I2C Module 0 Software Reset */
#  define SYSCON_SRI2C_R1              (1 << 1)   /* Bit 1:  I2C Module 1 Software Reset */
#  define SYSCON_SRI2C_R2              (1 << 2)   /* Bit 2:  I2C Module 2 Software Reset */
#  define SYSCON_SRI2C_R3              (1 << 3)   /* Bit 3:  I2C Module 3 Software Reset */
#  define SYSCON_SRI2C_R4              (1 << 4)   /* Bit 4:  I2C Module 4 Software Reset */
#  define SYSCON_SRI2C_R5              (1 << 5)   /* Bit 5:  I2C Module 5 Software Reset */
#  define SYSCON_SRI2C_R6              (1 << 6)   /* Bit 6:  I2C Module 6 Software Reset */
#  define SYSCON_SRI2C_R7              (1 << 7)   /* Bit 7:  I2C Module 7 Software Reset */
#  define SYSCON_SRI2C_R8              (1 << 8)   /* Bit 8:  I2C Module 8 Software Reset */
#  define SYSCON_SRI2C_R9              (1 << 9)   /* Bit 9:  I2C Module 9 Software Reset */

/* USB Software Reset */

#define SYSCON_SRUSB_R0                (1 << 0)   /* Bit 0:  USB Module Software Reset */

/* Ethernet PHY Software Reset */

#define SYSCON_SREPHY_R0               (1 << 0)   /* Bit 0: Ethernet PHY Module Software Reset */

/* CAN Software Reset */

#define SYSCON_SRCAN(n)                (1 << (n)) /* Bit n:  CAN Module n Software Reset */
#  define SYSCON_SRCAN_R0              (1 << 0)   /* Bit 0:  CAN Module 0 Software Reset */
#  define SYSCON_SRCAN_R1              (1 << 1)   /* Bit 1:  CAN Module 1 Software Reset*/

/* ADC Software Reset */

#define SYSCON_SRADC(n)                (1 << (n)) /* Bit n:  ADC Module n Software Reset */
#  define SYSCON_SRADC_R0              (1 << 0)   /* Bit 0:  ADC Module 0 Software Reset */
#  define SYSCON_SRADC_R1              (1 << 1)   /* Bit 1:  ADC Module 1 Software Reset */

/* ACMP Software Reset */

#define SYSCON_SRACMP_R0               (1 << 0)   /* Bit 0:  Analog Comparator Module 0 Software Reset */

/* PWM Software Reset */

#define SYSCON_SRPWM(n)                (1 << (n)) /* Bit n:  PWM Module n Software Reset */
#  define SYSCON_SRPWM_R0              (1 << 0)   /* Bit 0:  PWM Module 0 Software Reset */
#  define SYSCON_SRPWM_R1              (1 << 1)   /* Bit 1:  PWM Module 1 Software Reset */

/* QE Interface Software Reset */

#define SYSCON_SRQEI(n)                (1 << (n)) /* Bit n:  QEI Module n Software Reset */
#  define SYSCON_SRQEI_R0              (1 << 0)   /* Bit 0:  QEI Module 0 Software Reset */
#  define SYSCON_SRQEI_R1              (1 << 1)   /* Bit 1:  QEI Module 1 Software Reset */

/* EEPROM Software Reset */

#define SYSCON_SREEPROM_R0             (1 << 0)   /* Bit 0:  EEPROM Module Software Reset */

/* 32/64-Bit Wide Timer Software Reset */

#define SYSCON_SRWTIMER(n)             (1 << (n)) /* Bit n:  32/64-Bit Wide Timer n Software Reset */
#  define SYSCON_SRWTIMER_R0           (1 << 0)   /* Bit 0:  32/64-Bit Wide Timer 0 Software Reset */
#  define SYSCON_SRWTIMER_R1           (1 << 1)   /* Bit 1:  32/64-Bit Wide Timer 1 Software Reset */
#  define SYSCON_SRWTIMER_R2           (1 << 2)   /* Bit 2:  32/64-Bit Wide Timer 2 Software Reset */
#  define SYSCON_SRWTIMER_R3           (1 << 3)   /* Bit 3:  32/64-Bit Wide Timer 3 Software Reset */
#  define SYSCON_SRWTIMER_R4           (1 << 4)   /* Bit 4:  32/64-Bit Wide Timer 4 Software Reset */
#  define SYSCON_SRWTIMER_R5           (1 << 5)   /* Bit 5:  32/64-Bit Wide Timer 5 Software Reset */

/* CRC/Crypto Modules Software Reset */

#define SYSCON_SRCCM_R0                (1 << 0)   /* Bit 0: CRC/Crypto Modules Software Reset */

/* LCD Controller Software Reset */

#define SYSCON_SRLCD_R0                (1 << 0)   /* Bit 0: LCD Module 0 Software Reset */

/* 1-Wire Software Reset */

#define SYSCON_SROWIRE_R0              (1 << 0)   /* Bit 0: 1-Wire Module Software Reset */

/* Ethernet MAC Software Reset */

#define SYSCON_SREMAC_R0               (1 << 0)   /* Ethernet Controller MAC Module 0 Software Reset */

/* Watchdog Timer Run Mode Clock Gating Control */

#define SYSCON_RCGCWD(n)               (1 << (n)) /* Bit n:  Watchdog Timer n Run Mode Clock Gating Control */
#  define SYSCON_RCGCWD_R0             (1 << 0)   /* Bit 0:  Watchdog Timer 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCWD_R1             (1 << 1)   /* Bit 1:  Watchdog Timer 1 Run Mode Clock Gating Control */

/* 16/32-Bit Timer Run Mode Clock Gating Control */

#define SYSCON_RCGCTIMER(n)            (1 << (n)) /* Bit n:  16/32-Bit Timer n Run Mode Clock Gating Control */
#  define SYSCON_RCGCTIMER_R0          (1 << 0)   /* Bit 0:  16/32-Bit Timer 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCTIMER_R1          (1 << 1)   /* Bit 1:  16/32-Bit Timer 1 Run Mode Clock Gating Control */
#  define SYSCON_RCGCTIMER_R2          (1 << 2)   /* Bit 2:  16/32-Bit Timer 2 Run Mode Clock Gating Control */
#  define SYSCON_RCGCTIMER_R3          (1 << 3)   /* Bit 3:  16/32-Bit Timer 3 Run Mode Clock Gating Control */
#  define SYSCON_RCGCTIMER_R4          (1 << 4)   /* Bit 4:  16/32-Bit Timer 4 Run Mode Clock Gating Control */
#  define SYSCON_RCGCTIMER_R5          (1 << 5)   /* Bit 5:  16/32-Bit Timer 5 Run Mode Clock Gating Control */
#  define SYSCON_RCGCTIMER_R6          (1 << 6)   /* Bit 6:  16/32-Bit Timer 6 Run Mode Clock Gating Control */
#  define SYSCON_RCGCTIMER_R7          (1 << 7)   /* Bit 7:  16/32-Bit Timer 7 Run Mode Clock Gating Control */

/* GPIO Run Mode Clock Gating Control */

#define SYSCON_RCGCGPIO(n)             (1 << (n)) /* Bit n:  16/32-Bit GPIO Port n Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R0           (1 << 0)   /* Bit 0:  16/32-Bit GPIO Port A Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R1           (1 << 1)   /* Bit 1:  16/32-Bit GPIO Port B Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R2           (1 << 2)   /* Bit 2:  16/32-Bit GPIO Port C Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R3           (1 << 3)   /* Bit 3:  16/32-Bit GPIO Port D Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R4           (1 << 4)   /* Bit 4:  16/32-Bit GPIO Port E Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R5           (1 << 5)   /* Bit 5:  16/32-Bit GPIO Port F Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R6           (1 << 6)   /* Bit 6:  16/32-Bit GPIO Port G Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R7           (1 << 7)   /* Bit 7:  16/32-Bit GPIO Port H Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R8           (1 << 8)   /* Bit 8:  16/32-Bit GPIO Port J Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R9           (1 << 9)   /* Bit 9:  16/32-Bit GPIO Port K Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R10          (1 << 10)  /* Bit 10: 16/32-Bit GPIO Port L Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R11          (1 << 11)  /* Bit 11: 16/32-Bit GPIO Port M Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R12          (1 << 12)  /* Bit 12: 16/32-Bit GPIO Port N Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R13          (1 << 13)  /* Bit 13: 16/32-Bit GPIO Port P Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R14          (1 << 14)  /* Bit 14: 16/32-Bit GPIO Port Q Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R15          (1 << 15)  /* Bit 15: 16/32-Bit GPIO Port R Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R16          (1 << 16)  /* Bit 16: 16/32-Bit GPIO Port S Run Mode Clock Gating Control */
#  define SYSCON_RCGCGPIO_R17          (1 << 17)  /* Bit 17: 16/32-Bit GPIO Port T Run Mode Clock Gating Control */

/* μDMA Run Mode Clock Gating Control */

#define SYSCON_RCGCDMA_R0              (1 << 0)   /* Bit 0:  μDMA Module Run Mode Clock Gating Control */

/* EPI Run Mode Clock Gating Control */

#define SYSCON_RCGCEPI_R0              (1 << 0)   /* Bit 0: EPI Module Run Mode Clock Gating Control */

/* Hibernation Run Mode Clock Gating Control */

#define SYSCON_RCGCHIB_R0              (1 << 0)   /* Bit 0:  Hibernation Module Run Mode Clock Gating Control */

/* UART Run Mode Clock Gating Control */

#define SYSCON_RCGCUART(n)             (1 << (n)) /* Bit n:  UART Module n Run Mode Clock Gating Control */
#  define SYSCON_RCGCUART_R0           (1 << 0)   /* Bit 0:  UART Module 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCUART_R1           (1 << 1)   /* Bit 1:  UART Module 1 Run Mode Clock Gating Control */
#  define SYSCON_RCGCUART_R2           (1 << 2)   /* Bit 2:  UART Module 2 Run Mode Clock Gating Control */
#  define SYSCON_RCGCUART_R3           (1 << 3)   /* Bit 3:  UART Module 3 Run Mode Clock Gating Control */
#  define SYSCON_RCGCUART_R4           (1 << 4)   /* Bit 4:  UART Module 4 Run Mode Clock Gating Control */
#  define SYSCON_RCGCUART_R5           (1 << 5)   /* Bit 5:  UART Module 5 Run Mode Clock Gating Control */
#  define SYSCON_RCGCUART_R6           (1 << 6)   /* Bit 6:  UART Module 6 Run Mode Clock Gating Control */
#  define SYSCON_RCGCUART_R7           (1 << 7)   /* Bit 7:  UART Module 7 Run Mode Clock Gating Control */

/* SSI Run Mode Clock Gating Control */

#define SYSCON_RCGCSSI(n)              (1 << (n)) /* Bit n:  SSI Module n Run Mode Clock Gating Control */
#  define SYSCON_RCGCSSI_R0            (1 << 0)   /* Bit 0:  SSI Module 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCSSI_R1            (1 << 1)   /* Bit 1:  SSI Module 1 Run Mode Clock Gating Control */
#  define SYSCON_RCGCSSI_R2            (1 << 2)   /* Bit 2:  SSI Module 2 Run Mode Clock Gating Control */
#  define SYSCON_RCGCSSI_R3            (1 << 3)   /* Bit 3:  SSI Module 3 Run Mode Clock Gating Control */

/* I2C Run Mode Clock Gating Control */

#define SYSCON_RCGCI2C(n)              (1 << (n)) /* Bit n:  I2C Module n Run Mode Clock Gating Control */
#  define SYSCON_RCGCI2C_R0            (1 << 0)   /* Bit 0:  I2C Module 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCI2C_R1            (1 << 1)   /* Bit 1:  I2C Module 1 Run Mode Clock Gating Control */
#  define SYSCON_RCGCI2C_R2            (1 << 2)   /* Bit 2:  I2C Module 2 Run Mode Clock Gating Control */
#  define SYSCON_RCGCI2C_R3            (1 << 3)   /* Bit 3:  I2C Module 3 Run Mode Clock Gating Control */
#  define SYSCON_RCGCI2C_R4            (1 << 4)   /* Bit 4:  I2C Module 4 Run Mode Clock Gating Control */
#  define SYSCON_RCGCI2C_R5            (1 << 5)   /* Bit 5:  I2C Module 5 Run Mode Clock Gating Control */
#  define SYSCON_RCGCI2C_R6            (1 << 6)   /* Bit 6:  I2C Module 6 Run Mode Clock Gating Control */
#  define SYSCON_RCGCI2C_R7            (1 << 7)   /* Bit 7:  I2C Module 7 Run Mode Clock Gating Control */
#  define SYSCON_RCGCI2C_R8            (1 << 8)   /* Bit 8:  I2C Module 8 Run Mode Clock Gating Control */
#  define SYSCON_RCGCI2C_R9            (1 << 9)   /* Bit 9:  I2C Module 9 Run Mode Clock Gating Control */

/* USB Run Mode Clock Gating Control */

#define SYSCON_RCGCUSB_R0              (1 << 0)   /* Bit 0:  USB Module Run Mode Clock Gating Control */

/* Ethernet PHY Run Mode Clock Gating Control */

#define SYSCON_RCGCEPHY_R0             (1 << 0)   /* Bit 0:  Ethernet PHY Module Run Mode Clock Gating Control */

/* CAN RunMode Clock Gating Control */

#define SYSCON_RCGCCAN(n)              (1 << (n)) /* Bit n:  CAN Module n Run Mode Clock Gating Control */
#  define SYSCON_RCGCCAN_R0            (1 << 0)   /* Bit 0:  CAN Module 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCCAN_R1            (1 << 1)   /* Bit 1:  CAN Module 1 Run Mode Clock Gating Control */

/* ADC Run Mode Clock Gating Control */

#define SYSCON_RCGCADC(n)              (1 << (n)) /* Bit n:  ADC Module n Run Mode Clock Gating Control */
#  define SYSCON_RCGCADC_R0            (1 << 0)   /* Bit 0:  ADC Module 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCADC_R1            (1 << 1)   /* Bit 1:  ADC Module 1 Run Mode Clock Gating Control */

/* ACMP Run Mode Clock Gating Control */

#define SYSCON_RCGCACMP_R0             (1 << 0)   /* Bit 0:  Analog Comparator Module 0 Run Mode Clock Gating Control */

/* PWM Run Mode Clock Gating Control */

#define SYSCON_RCGCPWM(n)              (1 << (n)) /* Bit n:  PWM Module n Run Mode Clock Gating Control */
#  define SYSCON_RCGCPWM_R0            (1 << 0)   /* Bit 0:  PWM Module 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCPWM_R1            (1 << 1)   /* Bit 1:  PWM Module 1 Run Mode Clock Gating Control */

/* QE Interface Run Mode Clock Gating Control */

#define SYSCON_RCGCQEI(n)              (1 << (n)) /* Bit n:  QEI Module n Run Mode Clock Gating Control */
#  define SYSCON_RCGCQEI_R0            (1 << 0)   /* Bit 0:  QEI Module 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCQEI_R1            (1 << 1)   /* Bit 1:  QEI Module 1 Run Mode Clock Gating Control */

/* EEPROM Run Mode Clock Gating Control */

#define SYSCON_RCGCEEPROM_R0           (1 << 0)   /* Bit 0:  EEPROM Module Run Mode Clock Gating Control */

/* 32/64-Bit Wide Timer Run Mode Clock Gating Control */

#define SYSCON_RCGCWTIMER(n)           (1 << (n)) /* Bit n:  32/64-Bit Wide Timer n Run Mode Clock Gating Control */
#  define SYSCON_RCGCWTIMER_R0         (1 << 0)   /* Bit 0:  32/64-Bit Wide Timer 0 Run Mode Clock Gating Control */
#  define SYSCON_RCGCWTIMER_R1         (1 << 1)   /* Bit 1:  32/64-Bit Wide Timer 1 Run Mode Clock Gating Control */
#  define SYSCON_RCGCWTIMER_R2         (1 << 2)   /* Bit 2:  32/64-Bit Wide Timer 2 Run Mode Clock Gating Control */
#  define SYSCON_RCGCWTIMER_R3         (1 << 3)   /* Bit 3:  32/64-Bit Wide Timer 3 Run Mode Clock Gating Control */
#  define SYSCON_RCGCWTIMER_R4         (1 << 4)   /* Bit 4:  32/64-Bit Wide Timer 4 Run Mode Clock Gating Control */
#  define SYSCON_RCGCWTIMER_R5         (1 << 5)   /* Bit 5:  32/64-Bit Wide Timer 5 Run Mode Clock Gating Control */

/* CRC/Crypto Modules RunMode ClockGating Control */

#define SYSCON_RCGCCCM_R0              (1 << 0)   /* Bit 0:  CRC and Cryptographic Modules Run Mode Clock Gating Control */

/* LCD Controller Run Mode Clock Gating Control */

#define SYSCON_RCGCLCD_R0              (1 << 0)   /* Bit 0:  LCD Controller Module 0 Run Mode Clock Gating Control */

/* 1-Wire Run Mode Clock Gating Control */

#define SYSCON_RCGCOWIRE_R0            (1 << 0)   /* Bit 0:  1-Wire Module 0 Run Mode Clock Gating Control */

/* Ethernet MAC Run Mode Clock Gating Control */

#define SYSCON_RCGCEMAC_R0             (1 << 0)   /* Bit 0:  Ethernet MAC Module 0 Run Mode Clock Gating Control */

/* Watchdog Timer Sleep Mode Clock Gating Control */

#define SYSCON_SCGCWD(n)               (1 << (n)) /* Bit n:  Watchdog Timer n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWD_S0             (1 << 0)   /* Bit 0:  Watchdog Timer 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWD_S1             (1 << 1)   /* Bit 1:  Watchdog Timer 1 Sleep Mode Clock Gating Control */

/* 16/32-Bit Timer Sleep Mode Clock Gating Control */

#define SYSCON_SCGCWD(n)               (1 << (n)) /* Bit n:  16/32-Bit Timer n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWD_S0             (1 << 0)   /* Bit 0:  16/32-Bit Timer 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWD_S1             (1 << 1)   /* Bit 1:  16/32-Bit Timer 1 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWD_S2             (1 << 2)   /* Bit 2:  16/32-Bit Timer 2 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWD_S3             (1 << 3)   /* Bit 3:  16/32-Bit Timer 3 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWD_S4             (1 << 4)   /* Bit 4:  16/32-Bit Timer 4 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWD_S5             (1 << 5)   /* Bit 5:  16/32-Bit Timer 5 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWD_S6             (1 << 6)   /* Bit 6:  16/32-Bit Timer 6 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWD_S7             (1 << 7)   /* Bit 7:  16/32-Bit Timer 7 Sleep Mode Clock Gating Control */

/* GPIO Sleep Mode Clock Gating Control */

#define SYSCON_SCGCGPIO(n)             (1 << (n)) /* Bit n:  GPIO Port n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S0           (1 << 0)   /* Bit 0:  GPIO Port A Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S1           (1 << 1)   /* Bit 1:  GPIO Port B Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S2           (1 << 2)   /* Bit 2:  GPIO Port C Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S3           (1 << 3)   /* Bit 3:  GPIO Port D Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S4           (1 << 4)   /* Bit 4:  GPIO Port E Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S5           (1 << 5)   /* Bit 5:  GPIO Port F Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S6           (1 << 6)   /* Bit 6:  GPIO Port G Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S7           (1 << 7)   /* Bit 7:  GPIO Port H Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S8           (1 << 8)   /* Bit 8:  GPIO Port J Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S9           (1 << 9)   /* Bit 9:  GPIO Port K Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S10          (1 << 10)  /* Bit 10: GPIO Port L Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S11          (1 << 11)  /* Bit 11: GPIO Port M Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S12          (1 << 12)  /* Bit 12: GPIO Port N Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S13          (1 << 13)  /* Bit 13: GPIO Port P Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S14          (1 << 14)  /* Bit 14: GPIO Port Q Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S15          (1 << 15)  /* Bit 15: GPIO Port R Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S16          (1 << 16)  /* Bit 16: GPIO Port S Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCGPIO_S17          (1 << 17)  /* Bit 17: GPIO Port T Sleep Mode Clock Gating Control */

/* μDMA Sleep Mode Clock Gating Control */

#define SYSCON_SCGCDMA_S0              (1 << 0)   /* Bit 0:  μDMA Module Sleep Mode Clock Gating Control */

/* EPI Sleep Mode Clock Gating Control */

#define SYSCON_SCGCEPI_S0              (1 << 0)   /* Bit 0:  EPI Module Sleep Mode Clock Gating Control */

/* Hibernation Sleep Mode Clock Gating Control */

#define SYSCON_SCGCHIB_S0              (1 << 0)   /* Bit 0:  Hibernation Module Sleep Mode Clock Gating Control */

/* UART Sleep Mode Clock Gating Control */

#define SYSCON_SCGCUART(n)             (1 << (n)) /* Bit n:  UART Module n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCUART_S0           (1 << 0)   /* Bit 0:  UART Module 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCUART_S1           (1 << 1)   /* Bit 1:  UART Module 1 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCUART_S2           (1 << 2)   /* Bit 2:  UART Module 2 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCUART_S3           (1 << 3)   /* Bit 3:  UART Module 3 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCUART_S4           (1 << 4)   /* Bit 4:  UART Module 4 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCUART_S5           (1 << 5)   /* Bit 5:  UART Module 5 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCUART_S6           (1 << 6)   /* Bit 6:  UART Module 6 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCUART_S7           (1 << 7)   /* Bit 7:  UART Module 7 Sleep Mode Clock Gating Control */

/* SSI Sleep Mode Clock GatingControl */

#define SYSCON_SCGCSSI(n)              (1 << (n)) /* Bit n:  SSI Module n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCSSI_S0            (1 << 0)   /* Bit 0:  SSI Module 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCSSI_S1            (1 << 1)   /* Bit 1:  SSI Module 1 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCSSI_S2            (1 << 2)   /* Bit 2:  SSI Module 2 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCSSI_S3            (1 << 3)   /* Bit 3:  SSI Module 3 Sleep Mode Clock Gating Control */

/* I2C Sleep Mode Clock Gating Control */

#define SYSCON_SCGCI2C(n)              (1 << (n)) /* Bit n:  I2C Module n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCI2C_S0            (1 << 0)   /* Bit 0:  I2C Module 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCI2C_S1            (1 << 1)   /* Bit 1:  I2C Module 1 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCI2C_S2            (1 << 2)   /* Bit 2:  I2C Module 2 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCI2C_S3            (1 << 3)   /* Bit 3:  I2C Module 3 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCI2C_S4            (1 << 4)   /* Bit 4:  I2C Module 4 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCI2C_S5            (1 << 5)   /* Bit 5:  I2C Module 5 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCI2C_S6            (1 << 6)   /* Bit 6:  I2C Module 6 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCI2C_S7            (1 << 7)   /* Bit 7:  I2C Module 7 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCI2C_S8            (1 << 8)   /* Bit 8:  I2C Module 8 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCI2C_S9            (1 << 9)   /* Bit 9:  I2C Module 9 Sleep Mode Clock Gating Control */

/* USB Sleep Mode Clock Gating Control */

#define SYSCON_SCGCUSB_S0              (1 << 0)   /* Bit 0:  USB Module Sleep Mode Clock Gating Control */

/* Ethernet PHY Sleep Mode Clock Gating Control */

#define SYSCON_SCGCEPHY_S0             (1 << 0)   /* Bit 0: PHY Module Sleep Mode Clock Gating Control */

/* CAN Sleep Mode Clock Gating Control */

#define SYSCON_SCGCCAN(n)              (1 << (n)) /* Bit n:  CAN Module n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCCAN_S0            (1 << 0)   /* Bit 0:  CAN Module 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCCAN_S1            (1 << 1)   /* Bit 1:  CAN Module 1 Sleep Mode Clock Gating Control */

/* ADC Sleep Mode Clock Gating Control */

#define SYSCON_SCGCADC(n)              (1 << (n)) /* Bit n:  ADC Module n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCADC_S0            (1 << 0)   /* Bit 0:  ADC Module 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCADC_S1            (1 << 1)   /* Bit 1:  ADC Module 1 Sleep Mode Clock Gating Control */

/* ACMP Sleep Mode Clock Gating Control */

#define SYSCON_SCGCACMP_S0             (1 << 0)   /* Bit 0:  Analog Comparator Module 0 Sleep Mode Clock Gating Control */

/* PulseWidthModulator Sleep Mode Clock Gating Control */

#define SYSCON_SCGCPWM(n)              (1 << (n)) /* Bit n:  PWM Module n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCPWM_S0            (1 << 0)   /* Bit 0:  PWM Module 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCPWM_S1            (1 << 1)   /* Bit 1:  PWM Module 1 Sleep Mode Clock Gating Control */

/* QE Interface Sleep Mode Clock Gating Control */

#define SYSCON_SCGCQEI(n)              (1 << (n)) /* Bit n:  QEI Module n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCQEI_S0            (1 << 0)   /* Bit 0:  QEI Module 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCQEI_S1            (1 << 1)   /* Bit 1:  QEI Module 1 Sleep Mode Clock Gating Control */

/* EEPROM Sleep Mode Clock Gating Control */

#define SYSCON_SCGCEEPROM_S0           (1 << 0)   /* Bit 0:  EEPROM Module Sleep Mode Clock Gating Control */

/* 32/64-Bit Wide Timer Sleep Mode Clock Gating Control */

#define SYSCON_SCGCWTIMER(n)           (1 << (n)) /* Bit n:  32/64-Bit Wide Timer n Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWTIMER_S0         (1 << 0)   /* Bit 0:  32/64-Bit Wide Timer 0 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWTIMER_S1         (1 << 1)   /* Bit 1:  32/64-Bit Wide Timer 1 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWTIMER_S2         (1 << 2)   /* Bit 2:  32/64-Bit Wide Timer 2 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWTIMER_S3         (1 << 3)   /* Bit 3:  32/64-Bit Wide Timer 3 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWTIMER_S4         (1 << 4)   /* Bit 4:  32/64-Bit Wide Timer 4 Sleep Mode Clock Gating Control */
#  define SYSCON_SCGCWTIMER_S5         (1 << 5)   /* Bit 5:  32/64-Bit Wide Timer 5 Sleep Mode Clock Gating Control */

/* CRC/Crypto Modules Sleep Mode Clock Gating Control */

#define SYSCON_SCGCCCM_S0              (1 << 0)   /* Bit 0:  CRC and Cryptographic Modules Sleep Mode Clock Gating Control */

/* LCD Controller Sleep Mode Clock Gating Control */

#define SYSCON_SCGCLCD_S0              (1 << 0)   /* Bit 0: LCD Controller Module 0 Sleep Mode Clock Gating Control */

/* 1-Wire Sleep Mode Clock Gating Control */

#define SYSCON_SCGCOWIRE_S0            (1 << 0)   /* Bit 0: 1-Wire Module 0 Sleep Mode Clock Gating Control */

/* Ethernet MAC Sleep Mode Clock Gating Control */

#define SYSCON_SCGCEMAC_S0             (1 << 0)   /* Bit 0: Ethernet MAC Module 0 Sleep Mode Clock Gating Control */

/* Watchdog Timer Deep-SleepMode Clock Gating Control */

#define SYSCON_DCGCWD(n)               (1 << (n)) /* Bit n:  Watchdog Timer n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCWD_D0             (1 << 0)   /* Bit 0:  Watchdog Timer 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCWD_D1             (1 << 1)   /* Bit 1:  Watchdog Timer 1 Deep-Sleep Mode Clock Gating Control */

/* 16/32-Bit Timer Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCTIMER(n)            (1 << (n)) /* Bit n:  16/32-Bit Timer n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCTIMER_D0          (1 << 0)   /* Bit 0:  16/32-Bit Timer 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCTIMER_D1          (1 << 1)   /* Bit 1:  16/32-Bit Timer 1 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCTIMER_D2          (1 << 2)   /* Bit 2:  16/32-Bit Timer 2 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCTIMER_D3          (1 << 3)   /* Bit 3:  16/32-Bit Timer 3 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCTIMER_D4          (1 << 4)   /* Bit 4:  16/32-Bit Timer 4 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCTIMER_D5          (1 << 5)   /* Bit 5:  16/32-Bit Timer 5 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCTIMER_D6          (1 << 6)   /* Bit 6:  16/32-Bit Timer 6 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCTIMER_D7          (1 << 7)   /* Bit 7:  16/32-Bit Timer 7 Deep-Sleep Mode Clock Gating Control */

/* GPIO Deep-Sleep Mode Clock */

#define SYSCON_DCGCGPIO(n)             (1 << (n)) /* Bit n:  GPIO Port F Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D0           (1 << 0)   /* Bit 0:  GPIO Port A Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D1           (1 << 1)   /* Bit 1:  GPIO Port B Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D2           (1 << 2)   /* Bit 2:  GPIO Port C Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D3           (1 << 3)   /* Bit 3:  GPIO Port D Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D4           (1 << 4)   /* Bit 4:  GPIO Port E Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D5           (1 << 5)   /* Bit 5:  GPIO Port F Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D6           (1 << 6)   /* Bit 6:  GPIO Port G Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D7           (1 << 7)   /* Bit 7:  GPIO Port H Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D8           (1 << 8)   /* Bit 8:  GPIO Port J Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D9           (1 << 9)   /* Bit 9:  GPIO Port K Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D10          (1 << 10)  /* Bit 10: GPIO Port L Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D11          (1 << 11)  /* Bit 11: GPIO Port M Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D12          (1 << 12)  /* Bit 12: GPIO Port N Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D13          (1 << 13)  /* Bit 13: GPIO Port P Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D14          (1 << 14)  /* Bit 14: GPIO Port Q Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D15          (1 << 15)  /* Bit 15: GPIO Port R Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D16          (1 << 16)  /* Bit 16: GPIO Port S Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCGPIO_D17          (1 << 17)  /* Bit 17: GPIO Port T Deep-Sleep Mode Clock Gating Control */

/* μDMA Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCDMA_D0              (1 << 0)   /* Bit 0:  μDMA Module Deep-Sleep Mode Clock Gating Control */

/* EPI Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCEPI_D0              (1 << 0)   /* Bit 0:  EPI Module Deep-Sleep Mode Clock Gating Control */

/* Hibernation Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCHIB_D0              (1 << 0)   /* Bit 0:  Hibernation Module Deep-Sleep Mode Clock Gating Control */

/* UART Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCUART(n)             (1 << (n)) /* Bit n:  UART Module n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCUART_D0           (1 << 0)   /* Bit 0:  UART Module 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCUART_D1           (1 << 1)   /* Bit 1:  UART Module 1 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCUART_D2           (1 << 2)   /* Bit 2:  UART Module 2 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCUART_D3           (1 << 3)   /* Bit 3:  UART Module 3 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCUART_D4           (1 << 4)   /* Bit 4:  UART Module 4 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCUART_D5           (1 << 5)   /* Bit 5:  UART Module 5 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCUART_D6           (1 << 6)   /* Bit 6:  UART Module 6 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCUART_D7           (1 << 7)   /* Bit 7:  UART Module 7 Deep-Sleep Mode Clock Gating Control */

/* SSI Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCSSI(n)              (1 << (n)) /* Bit n:  SSI Module n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCSSI_D0            (1 << 0)   /* Bit 0:  SSI Module 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCSSI_D1            (1 << 1)   /* Bit 1:  SSI Module 1 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCSSI_D2            (1 << 2)   /* Bit 2:  SSI Module 2 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCSSI_D3            (1 << 3)   /* Bit 3:  SSI Module 3 Deep-Sleep Mode Clock Gating Control */

/* I2C Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCI2C(n)              (1 << (n)) /* Bit n:  I2C Module n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCI2C_D0            (1 << 0)   /* Bit 0:  I2C Module 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCI2C_D1            (1 << 1)   /* Bit 1:  I2C Module 1 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCI2C_D2            (1 << 2)   /* Bit 2:  I2C Module 2 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCI2C_D3            (1 << 3)   /* Bit 3:  I2C Module 3 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCI2C_D4            (1 << 4)   /* Bit 4:  I2C Module 4 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCI2C_D5            (1 << 5)   /* Bit 5:  I2C Module 5 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCI2C_D6            (1 << 6)   /* Bit 6:  I2C Module 6 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCI2C_D7            (1 << 7)   /* Bit 7:  I2C Module 7 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCI2C_D8            (1 << 8)   /* Bit 8:  I2C Module 8 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCI2C_D9            (1 << 9)   /* Bit 9:  I2C Module 9 Deep-Sleep Mode Clock Gating Control */

/* USB Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCUSB_D0              (1 << 0)   /* Bit 0:  USB Module Deep-Sleep Mode Clock Gating Control */

/* Ethernet PHY Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCEPHY_D0             (1 << 0)   /* Bit 0:  PHY Module Deep-Sleep Mode Clock Gating Control */

/* CAN Deep-SleepMode Clock Gating Control */

#define SYSCON_DCGCCAN(n)              (1 << (n)) /* Bit n:  CAN Module n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCCAN_D0            (1 << 0)   /* Bit 0:  CAN Module 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCCAN_D1            (1 << 1)   /* Bit 1:  CAN Module 1 Deep-Sleep Mode Clock Gating Control */

/* ADC Deep-Sleep Mode ClockGating Control */

#define SYSCON_DCGCADC(n)              (1 << (n)) /* Bit n:  ADC Module n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCADC_D0            (1 << 0)   /* Bit 0:  ADC Module 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCADC_D1            (1 << 1)   /* Bit 1:  ADC Module 1 Deep-Sleep Mode Clock Gating Control */

/* ACMP Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCACMP_D0             (1 << 0)   /* Bit 0:  Analog Comparator Module 0 Deep-Sleep Mode Clock Gating Control */

/* PWM Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCPWM(n)              (1 << (n)) /* Bit n:  PWM Module n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCPWM_D0            (1 << 0)   /* Bit 0:  PWM Module 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCPWM_D1            (1 << 1)   /* Bit 1:  PWM Module 1 Deep-Sleep Mode Clock Gating Control */

/* QE Interface Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCQEI(n)              (1 << (n)) /* Bit n:  QEI Module n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCQEI_D0            (1 << 0)   /* Bit 0:  QEI Module 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCQEI_D1            (1 << 1)   /* Bit 1:  QEI Module 1 Deep-Sleep Mode Clock Gating Control */

/* EEPROM Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCEEPROM_D0           (1 << 0)   /* Bit 0:  EEPROM Module Deep-Sleep Mode Clock Gating Control */

/* 32/64-Bit Wide Timer Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCWTIMER(n)          (1 << (n)) /* Bit n:  UART Module n Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCWTIMER_D0        (1 << 0)   /* Bit 0:  UART Module 0 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCWTIMER_D1        (1 << 1)   /* Bit 1:  UART Module 1 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCWTIMER_D2        (1 << 2)   /* Bit 2:  UART Module 2 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCWTIMER_D3        (1 << 3)   /* Bit 3:  UART Module 3 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCWTIMER_D4        (1 << 4)   /* Bit 4:  UART Module 4 Deep-Sleep Mode Clock Gating Control */
#  define SYSCON_DCGCWTIMER_D5        (1 << 5)   /* Bit 5:  UART Module 5 Deep-Sleep Mode Clock Gating Control */

/* CRC/Crypto Modules Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCCCM_D0              (1 << 0)   /* Bit 0:  CRC and Cryptographic Modules Deep-Sleep Mode Clock Gating Control */

/* LCD Controller Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCLCD_D0              (1 << 0)   /* Bit 0: LCD Controller Module 0 Deep-Sleep Mode Clock Gating Control */

/* 1-Wire Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCOWIRE_D0            (1 << 0)   /* Bit 0: 1-Wire Module 0 Deep-Sleep Mode Clock Gating Control */

/* Ethernet MAC Deep-Sleep Mode Clock Gating Control */

#define SYSCON_DCGCEMAC_D0             (1 << 0)   /* Bit 0:  Ethernet MAC Module 0 Deep-Sleep Mode Clock Gating Control */

/* Watchdog Timer Power Control */

#define SYSCON_PCWD(n)                 (1 << (n)) /* Bit n:  Watchdog Timer n Power Control */
#  define SYSCON_PCWD_P0               (1 << 0)   /* Bit 0:  Watchdog Timer 0 Power Control */
#  define SYSCON_PCWD_P1               (1 << 1)   /* Bit 1:  Watchdog Timer 1 Power Control */

/* 16/32-Bit Timer Power Control */

#define SYSCON_PCTIMER(n)              (1 << (n)) /* Bit n:  Timer n Power Control */
#  define SYSCON_PCTIMER_P7            (1 << 0)   /* Bit 0:  Timer 0 Power Control */
#  define SYSCON_PCTIMER_P6            (1 << 1)   /* Bit 1:  Timer 1 Power Control */
#  define SYSCON_PCTIMER_P5            (1 << 2)   /* Bit 2:  Timer 2 Power Control */
#  define SYSCON_PCTIMER_P4            (1 << 3)   /* Bit 3:  Timer 3 Power Control */
#  define SYSCON_PCTIMER_P3            (1 << 4)   /* Bit 4:  Timer 4 Power Control */
#  define SYSCON_PCTIMER_P2            (1 << 5)   /* Bit 5:  Timer 5 Power Control */
#  define SYSCON_PCTIMER_P1            (1 << 6)   /* Bit 6:  Timer 6 Power Control */
#  define SYSCON_PCTIMER_P0            (1 << 7)   /* Bit 7:  Timer 7 Power Control */

/* GPIO Power Control */

#define SYSCON_PCGPIO(n)               (1 << (n)) /* Bit n:  GPIO Port n Power Control */
#  define SYSCON_PCGPIO_P0             (1 << 0)   /* Bit 0:  GPIO Port A Power Control */
#  define SYSCON_PCGPIO_P1             (1 << 1)   /* Bit 1:  GPIO Port B Power Control */
#  define SYSCON_PCGPIO_P2             (1 << 2)   /* Bit 2:  GPIO Port C Power Control */
#  define SYSCON_PCGPIO_P3             (1 << 3)   /* Bit 3:  GPIO Port D Power Control */
#  define SYSCON_PCGPIO_P4             (1 << 4)   /* Bit 4:  GPIO Port E Power Control */
#  define SYSCON_PCGPIO_P5             (1 << 5)   /* Bit 5:  GPIO Port F Power Control */
#  define SYSCON_PCGPIO_P6             (1 << 6)   /* Bit 6:  GPIO Port G Power Control */
#  define SYSCON_PCGPIO_P7             (1 << 7)   /* Bit 7:  GPIO Port H Power Control */
#  define SYSCON_PCGPIO_P8             (1 << 8)   /* Bit 8:  GPIO Port J Power Control */
#  define SYSCON_PCGPIO_P9             (1 << 9)   /* Bit 9:  GPIO Port K Power Control */
#  define SYSCON_PCGPIO_P10            (1 << 10)  /* Bit 10: GPIO Port L Power Control */
#  define SYSCON_PCGPIO_P11            (1 << 11)  /* Bit 11: GPIO Port M Power Control */
#  define SYSCON_PCGPIO_P12            (1 << 12)  /* Bit 12: GPIO Port N Power Control */
#  define SYSCON_PCGPIO_P13            (1 << 13)  /* Bit 13: GPIO Port P Power Control */
#  define SYSCON_PCGPIO_P14            (1 << 14)  /* Bit 14: GPIO Port Q Power Control */
#  define SYSCON_PCGPIO_P15            (1 << 15)  /* Bit 15: GPIO Port R Power Control */
#  define SYSCON_PCGPIO_P16            (1 << 16)  /* Bit 16: GPIO Port S Power Control */
#  define SYSCON_PCGPIO_P17            (1 << 17)  /* Bit 17: GPIO Port T Power Control */

/* μDMA Power Control */

#define SYSCON_PCDMA_P0                (1 << 0)   /* Bit 0:  uDMA Module Power Control */

/* External Peripheral Interface Power Control */

#define SYSCON_PCEPI_P0                (1 << 0)   /* Bit 0:  EPI Module Power Control */

/* Hibernation Power Control */

#define SYSCON_PCHIB_P0                (1 << 0)   /* Bit 0:  Hibernation Module Power Control */

/* UART Power Control */

#define SYSCON_PCUART(n)               (1 << (n)) /* Bit n:  UART Module n Power Control */
#  define SYSCON_PCUART_P0             (1 << 0)   /* Bit 0:  UART Module 0 Power Control */
#  define SYSCON_PCUART_P1             (1 << 1)   /* Bit 1:  UART Module 1 Power Control */
#  define SYSCON_PCUART_P2             (1 << 2)   /* Bit 2:  UART Module 2 Power Control */
#  define SYSCON_PCUART_P3             (1 << 3)   /* Bit 3:  UART Module 3 Power Control */
#  define SYSCON_PCUART_P4             (1 << 4)   /* Bit 4:  UART Module 4 Power Control */
#  define SYSCON_PCUART_P5             (1 << 5)   /* Bit 5:  UART Module 5 Power Control */
#  define SYSCON_PCUART_P6             (1 << 6)   /* Bit 6:  UART Module 6 Power Control */
#  define SYSCON_PCUART_P7             (1 << 7)   /* Bit 7:  UART Module 7 Power Control */

/* SSI Power Control */

#define SYSCON_PCSSI(n)                (1 << (n)) /* Bit n:  SSI Module n Power Control */
#  define SYSCON_PCSSI_P0              (1 << 0)   /* Bit 0:  SSI Module 0 Power Control */
#  define SYSCON_PCSSI_P1              (1 << 1)   /* Bit 1:  SSI Module 1 Power Control */
#  define SYSCON_PCSSI_P2              (1 << 2)   /* Bit 2:  SSI Module 2 Power Control */
#  define SYSCON_PCSSI_P3              (1 << 3)   /* Bit 3:  SSI Module 3 Power Control */

/* I2C Power Control */

#define SYSCON_PCI2C(n)                (1 << (n)) /* Bit n:  I2C Module n Power Control */
#  define SYSCON_PCI2C_P0              (1 << 0)   /* Bit 0:  I2C Module 0 Power Control */
#  define SYSCON_PCI2C_P1              (1 << 1)   /* Bit 1:  I2C Module 1 Power Control */
#  define SYSCON_PCI2C_P2              (1 << 2)   /* Bit 2:  I2C Module 2 Power Control */
#  define SYSCON_PCI2C_P3              (1 << 3)   /* Bit 3:  I2C Module 3 Power Control */
#  define SYSCON_PCI2C_P4              (1 << 4)   /* Bit 4:  I2C Module 4 Power Control */
#  define SYSCON_PCI2C_P5              (1 << 5)   /* Bit 5:  I2C Module 5 Power Control */
#  define SYSCON_PCI2C_P6              (1 << 6)   /* Bit 6:  I2C Module 6 Power Control */
#  define SYSCON_PCI2C_P7              (1 << 7)   /* Bit 7:  I2C Module 7 Power Control */
#  define SYSCON_PCI2C_P8              (1 << 8)   /* Bit 8:  I2C Module 8 Power Control */
#  define SYSCON_PCI2C_P9              (1 << 9)   /* Bit 9:  I2C Module 9 Power Control */

/* USB Power Control */

#define SYSCON_PCUSB_P0                (1 << 0)   /* Bit 0:  USB Module Power Control */

/* Ethernet PHY Power Control */

#define SYSCON_PCEPHY_P0               (1 << 0)   /* Bit 0:  Ethernet PHY Module Power Control */

/* CAN Power Control */

#define SYSCON_PCCAN(n)                (1 << (n)) /* Bit n:  CAN Module n Power Control */
#  define SYSCON_PCCAN_P0              (1 << 0)   /* Bit 0:  CAN Module 0 Power Control */
#  define SYSCON_PCCAN_P1              (1 << 1)   /* Bit 1:  CAN Module 1 Power Control */

/* ADC Power Control */

#define SYSCON_PCADC(n)                (1 << (n)) /* Bit n:  ADC Module n Power Control */
#  define SYSCON_PCADC_P0              (1 << 0)   /* Bit 0:  ADC Module 0 Power Control */
#  define SYSCON_PCADC_P1              (1 << 1)   /* Bit 1:  ADC Module 1 Power Control */

/* ACMP Power Control */

#define SYSCON_PCACMP_P0               (1 << 0)   /* Bit 0:  Analog Comparator Module 0 Power Control */

/* PWM Power Control */

#define SYSCON_PCPWM(n)                (1 << (n)) /* Bit n:  PWM module n Power Control */
#  define SYSCON_PCPWM_P0              (1 << 0)   /* Bit 0:  PWM Module 0 Power Control */
#  define SYSCON_PCPWM_P1              (1 << 1)   /* Bit 1:  PWM Module 1 Power Control */

/* QE Interface Power Control */

#define SYSCON_PCQEI(n)                (1 << (n)) /* Bit n:  QEI module n Power Control */
#  define SYSCON_PCQEI_P0              (1 << 0)   /* Bit 0:  QEI Module 0 Power Control */
#  define SYSCON_PCQEI_P1              (1 << 1)   /* Bit 1:  QEI Module 1 Power Control */

/* EEPROM Power Control */

#define SYSCON_PCEEPROM_P0             (1 << 0)   /* Bit 0:  EEPROM Module 0 Power Control */

/* CRC/Crypto Modules Power Control */

#define SYSCON_PCCCM_P0                (1 << 0)   /* Bit 0:  CRC and Cryptographic Modules Power Control */

/* LCD Controller Power Control */

#define SYSCON_PCLCD_P0                (1 << 0)   /* Bit 0:  LCD Controller Module 0 Power Control */

/* 1-Wire Power Control */

#define SYSCON_PCOWIRE_P0              (1 << 0)   /* Bit 0:  1-Wire Module 0 Power Control */

/* Ethernet MAC Power Control */

#define SYSCON_PCEMAC_P0               (1 << 0)   /* Bit 0:  Ethernet MAC Module 0 Power Control */

/* Watchdog Timer Peripheral Ready */

#define SYSCON_PRWD(n)                 (1 << (n)) /* Bit n:  Watchdog Timer n Peripheral Ready */
#  define SYSCON_PRWD_R0               (1 << 0)   /* Bit 0:  Watchdog Timer 0 Peripheral Ready */
#  define SYSCON_PRWD_R1               (1 << 1)   /* Bit 1:  Watchdog Timer 1 Peripheral Ready */

/* 16/32-Bit Timer Peripheral Ready */

#define SYSCON_PRTIMER(n)              (1 << (n)) /* Bit n:  16/32-Bit Timer n Peripheral Ready */
#  define SYSCON_PRTIMER_R0            (1 << 0)   /* Bit 0:  16/32-Bit Timer 0 Peripheral Ready */
#  define SYSCON_PRTIMER_R1            (1 << 1)   /* Bit 1:  16/32-Bit Timer 1 Peripheral Ready */
#  define SYSCON_PRTIMER_R2            (1 << 2)   /* Bit 2:  16/32-Bit Timer 2 Peripheral Ready */
#  define SYSCON_PRTIMER_R3            (1 << 3)   /* Bit 3:  16/32-Bit Timer 3 Peripheral Ready */
#  define SYSCON_PRTIMER_R4            (1 << 4)   /* Bit 4:  16/32-Bit Timer 4 Peripheral Ready */
#  define SYSCON_PRTIMER_R5            (1 << 5)   /* Bit 5:  16/32-Bit Timer 5 Peripheral Ready */
#  define SYSCON_PRTIMER_R6            (1 << 6)   /* Bit 6:  16/32-Bit Timer 6 Peripheral Ready */
#  define SYSCON_PRTIMER_R7            (1 << 7)   /* Bit 7:  16/32-Bit Timer 7 Peripheral Ready */

/* GPIO Peripheral Ready */

#define SYSCON_PRGPIO(n)               (1 << (n)) /* Bit n:  GPIO Port n Peripheral Ready */
#  define SYSCON_PRGPIO_R0             (1 << 0)   /* Bit 0:  GPIO Port A Peripheral Ready */
#  define SYSCON_PRGPIO_R1             (1 << 1)   /* Bit 1:  GPIO Port B Peripheral Ready */
#  define SYSCON_PRGPIO_R2             (1 << 2)   /* Bit 2:  GPIO Port C Peripheral Ready */
#  define SYSCON_PRGPIO_R3             (1 << 3)   /* Bit 3:  GPIO Port D Peripheral Ready */
#  define SYSCON_PRGPIO_R4             (1 << 4)   /* Bit 4:  GPIO Port E Peripheral Ready */
#  define SYSCON_PRGPIO_R5             (1 << 5)   /* Bit 5:  GPIO Port F Peripheral Ready */
#  define SYSCON_PRGPIO_R6             (1 << 6)   /* Bit 6:  GPIO Port G Peripheral Ready */
#  define SYSCON_PRGPIO_R7             (1 << 7)   /* Bit 7:  GPIO Port H Peripheral Ready */
#  define SYSCON_PRGPIO_R8             (1 << 8)   /* Bit 8:  GPIO Port J Peripheral Ready */
#  define SYSCON_PRGPIO_R9             (1 << 9)   /* Bit 9:  GPIO Port K Peripheral Ready */
#  define SYSCON_PRGPIO_R10            (1 << 10)  /* Bit 10: GPIO Port L Peripheral Ready */
#  define SYSCON_PRGPIO_R11            (1 << 11)  /* Bit 11: GPIO Port M Peripheral Ready */
#  define SYSCON_PRGPIO_R12            (1 << 12)  /* Bit 12: GPIO Port N Peripheral Ready */
#  define SYSCON_PRGPIO_R13            (1 << 13)  /* Bit 13: GPIO Port P Peripheral Ready */
#  define SYSCON_PRGPIO_R14            (1 << 14)  /* Bit 14: GPIO Port Q Peripheral Ready */
#  define SYSCON_PRGPIO_R15            (1 << 15)  /* Bit 15: GPIO Port R Peripheral Ready */
#  define SYSCON_PRGPIO_R16            (1 << 16)  /* Bit 16: GPIO Port S Peripheral Ready */
#  define SYSCON_PRGPIO_R17            (1 << 17)  /* Bit 17: GPIO Port T Peripheral Ready */

/* μDMA Peripheral Ready */

#define SYSCON_PRDMA_R0                (1 << 0)   /* Bit 0:  μDMA Module Peripheral Ready */

/* EPI Peripheral Ready */

#define SYSCON_PREPI_R0                (1 << 0)   /* Bit 0:  EPI Module Peripheral Ready */

/* Hibernation Peripheral Ready */

#define SYSCON_PRHIB_R0                (1 << 0)   /* Bit 0:  Hibernation Module Peripheral Ready */

/* UART Peripheral Ready */

#define SYSCON_PRUART(n)               (1 << (n)) /* Bit n:  UART Module n Peripheral Ready */
#  define SYSCON_PRUART_R0             (1 << 0)   /* Bit 0:  UART Module 0 Peripheral Ready */
#  define SYSCON_PRUART_R1             (1 << 1)   /* Bit 1:  UART Module 1 Peripheral Ready */
#  define SYSCON_PRUART_R2             (1 << 2)   /* Bit 2:  UART Module 2 Peripheral Ready */
#  define SYSCON_PRUART_R3             (1 << 3)   /* Bit 3:  UART Module 3 Peripheral Ready */
#  define SYSCON_PRUART_R4             (1 << 4)   /* Bit 4:  UART Module 4 Peripheral Ready */
#  define SYSCON_PRUART_R5             (1 << 5)   /* Bit 5:  UART Module 5 Peripheral Ready */
#  define SYSCON_PRUART_R6             (1 << 6)   /* Bit 6:  UART Module 6 Peripheral Ready */
#  define SYSCON_PRUART_R7             (1 << 7)   /* Bit 7:  UART Module 7 Peripheral Ready */

/* SSI Peripheral Ready */

#define SYSCON_PRSSI(n)                (1 << (n)) /* Bit n:  SSI Module n Peripheral Ready */
#  define SYSCON_PRSSI_R0              (1 << 0)   /* Bit 0:  SSI Module 0 Peripheral Ready */
#  define SYSCON_PRSSI_R1              (1 << 1)   /* Bit 1:  SSI Module 1 Peripheral Ready */
#  define SYSCON_PRSSI_R2              (1 << 2)   /* Bit 2:  SSI Module 2 Peripheral Ready */
#  define SYSCON_PRSSI_R3              (1 << 3)   /* Bit 3:  SSI Module 3 Peripheral Ready */

/* I2C Peripheral Ready */

#define SYSCON_PRI2C(n)                (1 << (n)) /* Bit n:  I2C Module n Peripheral Ready */
#  define SYSCON_PRI2C_R0              (1 << 0)   /* Bit 0:  I2C Module 0 Peripheral Ready */
#  define SYSCON_PRI2C_R1              (1 << 1)   /* Bit 1:  I2C Module 1 Peripheral Ready */
#  define SYSCON_PRI2C_R2              (1 << 2)   /* Bit 2:  I2C Module 2 Peripheral Ready */
#  define SYSCON_PRI2C_R3              (1 << 3)   /* Bit 3:  I2C Module 3 Peripheral Ready */
#  define SYSCON_PRI2C_R4              (1 << 4)   /* Bit 4:  I2C Module 4 Peripheral Ready */
#  define SYSCON_PRI2C_R5              (1 << 5)   /* Bit 5:  I2C Module 5 Peripheral Ready */
#  define SYSCON_PRI2C_R6              (1 << 6)   /* Bit 6:  I2C Module 5 Peripheral Ready */
#  define SYSCON_PRI2C_R7              (1 << 7)   /* Bit 7:  I2C Module 5 Peripheral Ready */
#  define SYSCON_PRI2C_R8              (1 << 8)   /* Bit 8:  I2C Module 5 Peripheral Ready */
#  define SYSCON_PRI2C_R9              (1 << 9)   /* Bit 9:  I2C Module 5 Peripheral Ready */

/* USB Peripheral Ready */

#define SYSCON_PRUSB_R0                (1 << 0)   /* Bit 0:  USB Module Peripheral Ready */

/* Ethernet PHY Peripheral Ready */

#define SYSCON_PREPHY_R0               (1 << 0)   /* Bit 0: Ethernet PHY Module Peripheral Ready */

/* CAN Peripheral Ready */

#define SYSCON_PRCAN(n)                (1 << (n)) /* Bit n:  CAN Module n Peripheral Ready */
#  define SYSCON_PRCAN_R0              (1 << 0)   /* Bit 0:  CAN Module 0 Peripheral Ready */
#  define SYSCON_PRCAN_R1              (1 << 1)   /* Bit 1:  CAN Module 1 Peripheral Ready */

/* ADC Peripheral Ready */

#define SYSCON_PRADC(n)                (1 << (n)) /* Bit n:  ADC Module n Peripheral Ready */
#  define SYSCON_PRADC_R0              (1 << 0)   /* Bit 0:  ADC Module 0 Peripheral Ready */
#  define SYSCON_PRADC_R1              (1 << 1)   /* Bit 1:  ADC Module 1 Peripheral Ready */

/* ACMP Peripheral Ready */

#define SYSCON_PRACMP_R0               (1 << 0)   /* Bit 0:  Analog Comparator Module 0 Peripheral Ready */

/* PWM Peripheral Ready */

#define SYSCON_PRPWM(n)                (1 << (n)) /* Bit n:  PWM Module n Peripheral Ready */
#  define SYSCON_PRPWM_R0              (1 << 0)   /* Bit 0:  PWM Module 0 Peripheral Ready */
#  define SYSCON_PRPWM_R1              (1 << 1)   /* Bit 1:  PWM Module 1 Peripheral Ready */

/* QE Interface Peripheral Ready */

#define SYSCON_PRQEI(n)                (1 << (n)) /* Bit n:  QEI Module n Peripheral Ready */
#  define SYSCON_PRQEI_R0              (1 << 0)   /* Bit 0:  QEI Module 0 Peripheral Ready */
#  define SYSCON_PRQEI_R1              (1 << 1)   /* Bit 1:  QEI Module 1 Peripheral Ready */

/* EEPROM Peripheral Ready */

#define SYSCON_PREEPROM_0              (1 << 0)   /* Bit 0:  EEPROM Module Peripheral Ready */

/* 32/64-Bit Wide Timer Peripheral Ready */

#define SYSCON_PRWTIMER(n)            (1 << (n)) /* Bit n:  32/64-Bit Wide Timer n Peripheral Ready */
#  define SYSCON_PRWTIMER_R0          (1 << 0)   /* Bit 0:  32/64-Bit Wide Timer 0 Peripheral Ready */
#  define SYSCON_PRWTIMER_R1          (1 << 1)   /* Bit 1:  32/64-Bit Wide Timer 1 Peripheral Ready */
#  define SYSCON_PRWTIMER_R2          (1 << 2)   /* Bit 2:  32/64-Bit Wide Timer 2 Peripheral Ready */
#  define SYSCON_PRWTIMER_R3          (1 << 3)   /* Bit 3:  32/64-Bit Wide Timer 3 Peripheral Ready */
#  define SYSCON_PRWTIMER_R4          (1 << 4)   /* Bit 4:  32/64-Bit Wide Timer 4 Peripheral Ready */
#  define SYSCON_PRWTIMER_R5          (1 << 5)   /* Bit 5:  32/64-Bit Wide Timer 5 Peripheral Ready */

/* CRC/Crypto Modules Peripheral Ready */

#define SYSCON_PRCCM_R0                (1 << 0)   /* Bit 0:  CRC and Cryptographic Modules Peripheral Ready */

/* LCD Controller Peripheral Ready */

#define SYSCON_PRLCD_R0                (1 << 0)   /* Bit 0:  LCD Controller Module 0 Peripheral Ready */

/* 1-Wire Peripheral Ready */

#define SYSCON_PROWIRE_R0              (1 << 0)   /* Bit 0:  1-Wire Module 0 Peripheral Ready */

/* Ethernet MAC Peripheral Ready */

#define SYSCON_PREMAC_R0               (1 << 0)   /* Bit 0:  Ethernet MAC Module 0 Peripheral Ready */

/* Unique ID 0-3: 32-bit values */

/* CCM System Control Registers (CCM Control Offset) */

/* Cryptographic Modules Clock Gating Request */

#define SYSCON_CCMCGREQ_SHACFG         (1 << 0)   /* Bit 0:  SHA/MD5 Clock Gating Request */
#define SYSCON_CCMCGREQ_AESCFG         (1 << 1)   /* Bit 1:  AES Clock Gating Request */
#define SYSCON_CCMCGREQ_DESCFG         (1 << 2)   /* Bit 2:  DES Clock Gating Request */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_TM4C_TM4C129_SYSCTRL_H */

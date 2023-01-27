/****************************************************************************
 * arch/arm64/include/a64/irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM64_INCLUDE_A64_IRQ_H
#define __ARCH_ARM64_INCLUDE_A64_IRQ_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allwinner A64 Interrupts */

#define NR_IRQS                 157   /* Total number of interrupts */

#define A64_IRQ_SGI0            (0)   /* 0x0000 SGI 0 interrupt */
#define A64_IRQ_SGI1            (1)   /* 0x0004 SGI 1 interrupt */
#define A64_IRQ_SGI2            (2)   /* 0x0008 SGI 2 interrupt */
#define A64_IRQ_SGI3            (3)   /* 0x000C SGI 3 interrupt */
#define A64_IRQ_SGI4            (4)   /* 0x0010 SGI 4 interrupt */
#define A64_IRQ_SGI5            (5)   /* 0x0014 SGI 5 interrupt */
#define A64_IRQ_SGI6            (6)   /* 0x0018 SGI 6 interrupt */
#define A64_IRQ_SGI7            (7)   /* 0x001C SGI 7 interrupt */
#define A64_IRQ_SGI8            (8)   /* 0x0020 SGI 8 interrupt */
#define A64_IRQ_SGI9            (9)   /* 0x0024 SGI 9 interrupt */
#define A64_IRQ_SGI10           (10)  /* 0x0028 SGI 10 interrupt */
#define A64_IRQ_SGI11           (11)  /* 0x002C SGI 11 interrupt */
#define A64_IRQ_SGI12           (12)  /* 0x0030 SGI 12 interrupt */
#define A64_IRQ_SGI13           (13)  /* 0x0034 SGI 13 interrupt */
#define A64_IRQ_SGI14           (14)  /* 0x0038 SGI 14 interrupt */
#define A64_IRQ_SGI15           (15)  /* 0x003C SGI 15 interrupt */
#define A64_IRQ_PPI0            (16)  /* 0x0040 PPI 0 interrupt */
#define A64_IRQ_PPI1            (17)  /* 0x0044 PPI 1 interrupt */
#define A64_IRQ_PPI2            (18)  /* 0x0048 PPI 2 interrupt */
#define A64_IRQ_PPI3            (19)  /* 0x004C PPI 3 interrupt */
#define A64_IRQ_PPI4            (20)  /* 0x0050 PPI 4 interrupt */
#define A64_IRQ_PPI5            (21)  /* 0x0054 PPI 5 interrupt */
#define A64_IRQ_PPI6            (22)  /* 0x0058 PPI 6 interrupt */
#define A64_IRQ_PPI7            (23)  /* 0x005C PPI 7 interrupt */
#define A64_IRQ_PPI8            (24)  /* 0x0060 PPI 8 interrupt */
#define A64_IRQ_PPI9            (25)  /* 0x0064 PPI 9 interrupt */
#define A64_IRQ_PPI10           (26)  /* 0x0068 PPI 10 interrupt */
#define A64_IRQ_PPI11           (27)  /* 0x006C PPI 11 interrupt */
#define A64_IRQ_PPI12           (28)  /* 0x0070 PPI 12 interrupt */
#define A64_IRQ_PPI13           (29)  /* 0x0074 PPI 13 interrupt */
#define A64_IRQ_PPI14           (30)  /* 0x0078 PPI 14 interrupt */
#define A64_IRQ_PPI15           (31)  /* 0x007C PPI 15 interrupt */
#define A64_IRQ_UART0           (32)  /* 0x0080 UART 0 interrupt */
#define A64_IRQ_UART1           (33)  /* 0x0084 UART 1 interrupt */
#define A64_IRQ_UART2           (34)  /* 0x0088 UART 2 interrupt */
#define A64_IRQ_UART3           (35)  /* 0x008C UART 3 interrupt */
#define A64_IRQ_UART4           (36)  /* 0x0090 UART 4 interrupt */

#define A64_IRQ_TWI0            (38)  /* 0x0098 TWI 0 interrupt */
#define A64_IRQ_TWI1            (39)  /* 0x009C TWI 1 interrupt */
#define A64_IRQ_TWI2            (40)  /* 0x00A0 TWI 2 interrupt */

#define A64_IRQ_PB_EINT         (43)  /* 0x00AC PB_EINT interrupt */
#define A64_IRQ_OWA             (44)  /* 0x00B0 OWA interrupt */
#define A64_IRQ_I2S_PCM0        (45)  /* 0x00B4 I2S/PCM-0 interrupt */
#define A64_IRQ_I2S_PCM1        (46)  /* 0x00B8 I2S/PCM-1 interrupt */
#define A64_IRQ_I2S_PCM2        (47)  /* 0x00BC I2S/PCM-2 interrupt */

#define A64_IRQ_PG_EINT         (49)  /* 0x00C4 PG_EINT interrupt */
#define A64_IRQ_TIMER0          (50)  /* 0x00C8 Timer 0 interrupt */
#define A64_IRQ_TIMER1          (51)  /* 0x00CC Timer 1 interrupt */

#define A64_IRQ_PH_EINT         (53)  /* 0x00C4 PH_EINT interrupt */

#define A64_IRQ_AC_DET          (60)  /* 0x00F0 Audio Codec earphone detect interrupt */
#define A64_IRQ_AUDIO_CODEC     (61)  /* 0x00F4 Audio Codec interrupt */
#define A64_IRQ_KEYADC          (62)  /* 0x00F8 KEYADC interrupt */
#define A64_IRQ_THERMAL_SENSOR  (63)  /* 0x00FC Thermal Sensor interrupt */
#define A64_IRQ_EXTERNAL_NMI    (64)  /* 0x100 External Non-Mask Interrupt */
#define A64_IRQ_R_TIMER0        (65)  /* 0x104 R_timer 0 interrupt */
#define A64_IRQ_R_TIMER1        (66)  /* 0x108 R_timer 1 interrupt */

#define A64_IRQ_R_WATCHDOG      (68)  /* 0x0110 R_watchdog interrupt */
#define A64_IRQ_R_CIR_RX        (69)  /* 0x00E8 R_CIR_RX interrupt */
#define A64_IRQ_R_UART          (70)  /* 0x0118 R_UART interrupt */
#define A64_IRQ_R_RSB           (71)  /* 0x011C R_RSB interrupt */
#define A64_IRQ_R_ALARM0        (72)  /* 0x0120 R_Alarm 0 interrupt */
#define A64_IRQ_R_ALARM1        (73)  /* 0x0124 R_Alarm 1 interrupt */
#define A64_IRQ_R_TIMER2        (74)  /* 0x0128 R_timer 2 interrupt */
#define A64_IRQ_R_TIMER3        (75)  /* 0x012C R_timer 3 interrupt */
#define A64_IRQ_R_TWI           (76)  /* 0x0130 R_TWI interrupt */
#define A64_IRQ_R_PL_EINT       (77)  /* 0x0134 R_PL_EINT interrupt */
#define A64_IRQ_R_TWD           (78)  /* 0x0138 R_TWD interrupt */

#define A64_IRQ_MSGBOX          (81)  /* 0x0144 MSGBOX interrupt */
#define A64_IRQ_DMA             (82)  /* 0x0148 DMA channel interrupt */
#define A64_IRQ_HS_TIMER        (83)  /* 0x014C HS Timer interrupt */

#define A64_IRQ_SD_MMC0         (92)  /* 0x0170 SD/MMC Host Controller 0 interrupt */
#define A64_IRQ_SD_MMC1         (93)  /* 0x0174 SD/MMC Host Controller 1 interrupt */
#define A64_IRQ_SD_MMC2         (94)  /* 0x0178 SD/MMC Host Controller 2 interrupt */

#define A64_IRQ_SPI0            (97)  /* 0x0184 SPI 0 interrupt */
#define A64_IRQ_SPI1            (98)  /* 0x0188 SPI 1 interrupt */

#define A64_IRQ_DRAM_MDFS       (101) /* 0x0194 DRAM MDFS interrupt */
#define A64_IRQ_NAND            (102) /* 0x0198 NAND Flash Controller interrupt */
#define A64_IRQ_USB_OTG         (103) /* 0x019C USB-OTG interrupt */
#define A64_IRQ_USB_OTG_EHCI    (104) /* 0x01A0 USB-OTG-EHCI interrupt */
#define A64_IRQ_USB_OTG_OHCI    (105) /* 0x01A4 USB-OTG-OHCI interrupt */
#define A64_IRQ_USB_EHCI0       (106) /* 0x01A8 USB-EHCI0 interrupt */
#define A64_IRQ_USB_OHCI0       (107) /* 0x01AC USB-OHCI0 interrupt */

#define A64_IRQ_CE0             (112) /* 0x01C0 CE interrupt */
#define A64_IRQ_TS              (113) /* 0x01C4 TS interrupt */
#define A64_IRQ_EMAC            (114) /* 0x01C8 EMAC interrupt */
#define A64_IRQ_SCR             (115) /* 0x01CC SCR interrupt */
#define A64_IRQ_CSI             (116) /* 0x01D0 CSI interrupt */
#define A64_IRQ_CSI_CCI         (117) /* 0x01D4 CSI_CCI interrupt */
#define A64_IRQ_TCON0           (118) /* 0x01D8 TCON0 interrupt */
#define A64_IRQ_TCON1           (119) /* 0x01DC TCON1 interrupt */
#define A64_IRQ_HDMI            (120) /* 0x01E0 HDMI interrupt */
#define A64_IRQ_MIPI_DSI        (121) /* 0x01E4 MIPI DSI interrupt */

#define A64_IRQ_DIT             (125) /* 0x01F4 De-interlace interrupt */
#define A64_IRQ_CE1             (126) /* 0x01F8 CE1 interrupt */
#define A64_IRQ_DE              (127) /* 0x01FC DE interrupt */
#define A64_IRQ_ROT             (128) /* 0x0200 DE_RORATE interrupt */
#define A64_IRQ_GPU_GP          (129) /* 0x0204 GPU-GP interrupt */
#define A64_IRQ_GPU_GPMMU       (130) /* 0x0208 GPU-GPMMU interrupt */
#define A64_IRQ_GPU_PP0         (131) /* 0x020C GPU-PP0 interrupt */
#define A64_IRQ_GPU_PP0MMU      (132) /* 0x0210 GPU-PPMMU0 interrupt */
#define A64_IRQ_GPU_PMU         (133) /* 0x0214 GPU-PMU interrupt */
#define A64_IRQ_GPU_PP1         (134) /* 0x0218 GPU-PP1 interrupt */
#define A64_IRQ_GPU_PPMMU1      (135) /* 0x021C GPU-PPMMU1 interrupt */

#define A64_IRQ_CTI0            (140) /* 0x0230 CTI0 interrupt */
#define A64_IRQ_CTI1            (141) /* 0x0234 CTI1 interrupt */
#define A64_IRQ_CTI2            (142) /* 0x0238 CTI2 interrupt */
#define A64_IRQ_CTI3            (143) /* 0x023C CTI3 interrupt */
#define A64_IRQ_COMMTX0         (144) /* 0x0240 COMMTX0 interrupt */
#define A64_IRQ_COMMTX1         (145) /* 0x0244 COMMTX1 interrupt */
#define A64_IRQ_COMMTX2         (146) /* 0x0248 COMMTX2 interrupt */
#define A64_IRQ_COMMTX3         (147) /* 0x024C COMMTX3 interrupt */
#define A64_IRQ_COMMRX0         (148) /* 0x0250 COMMRX0 interrupt */
#define A64_IRQ_COMMRX1         (149) /* 0x0254 COMMRX1 interrupt */
#define A64_IRQ_COMMRX2         (150) /* 0x0258 COMMRX2 interrupt */
#define A64_IRQ_COMMRX3         (151) /* 0x025C COMMRX3 interrupt */
#define A64_IRQ_PMU0            (152) /* 0x0260 PMU0 interrupt */
#define A64_IRQ_PMU1            (153) /* 0x0264 PMU1 interrupt */
#define A64_IRQ_PMU2            (154) /* 0x0268 PMU2 interrupt */
#define A64_IRQ_PMU3            (155) /* 0x026C PMU3 interrupt */
#define A64_IRQ_AXI_ERROR       (156) /* 0x0270 AXI_ERROR interrupt */

#endif /* __ARCH_ARM64_INCLUDE_A64_IRQ_H */

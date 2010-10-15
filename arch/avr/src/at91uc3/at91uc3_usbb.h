/************************************************************************************
 * arch/avr/src/at91uc3/at91uc3_usbb.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************************************/

#ifndef __ARCH_AVR_SRC_AT91UC3_AT91UC3_USBB_H
#define __ARCH_AVR_SRC_AT91UC3_AT91UC3_USBB_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Register offsets *****************************************************************/

#define AVR32_USBB_UDCON_OFFSET       0x0000 /* Device General Control Register */
#define AVR32_USBB_UDINT_OFFSET       0x0004 /* Device Global Interrupt Register */
#define AVR32_USBB_UDINTCLR_OFFSET    0x0008 /* Device Global Interrupt Clear Register */
#define AVR32_USBB_UDINTSET_OFFSET    0x000c /* Device Global Interrupt Set Register */
#define AVR32_USBB_UDINTE_OFFSET      0x0010 /* Device Global Interrupt Enable Register */
#define AVR32_USBB_UDINTECLR_OFFSET   0x0014 /* Device Global Interrupt Enable Clear Register */
#define AVR32_USBB_UDINTESET_OFFSET   0x0018 /* Device Global Interrupt Enable Set Register */
#define AVR32_USBB_UERST_OFFSET       0x001c /* Endpoint Enable/Reset Register */
#define AVR32_USBB_UDFNUM_OFFSET      0x0020 /* Device Frame Number Register */

#define AVR32_USBB_UECFG_OFFSET(n)    (0x0100+((n)<<2))
#define AVR32_USBB_UECFG0_OFFSET      0x0100 /* Endpoint 0 Configuration Register */
#define AVR32_USBB_UECFG1_OFFSET      0x0104 /* Endpoint 1 Configuration Register */
#define AVR32_USBB_UECFG2_OFFSET      0x0108 /* Endpoint 2 Configuration Register */
#define AVR32_USBB_UECFG3_OFFSET      0x010c /* Endpoint 3 Configuration Register */
#define AVR32_USBB_UECFG4_OFFSET      0x0110 /* Endpoint 4 Configuration Register */
#define AVR32_USBB_UECFG5_OFFSET      0x0114 /* Endpoint 5 Configuration Register */
#define AVR32_USBB_UECFG6_OFFSET      0x0118 /* Endpoint 6 Configuration Register */
#define AVR32_USBB_UECFG7OFFSET       0x011c /* Endpoint 7 Configuration Register */

#define AVR32_USBB_UESTA_OFFSET(n)    (0x0130+((n)<<2))
#define AVR32_USBB_UESTA0_OFFSET      0x0130 /* Endpoint 0 Status Register */
#define AVR32_USBB_UESTA1_OFFSET      0x0134 /* Endpoint 1 Status Register */
#define AVR32_USBB_UESTA2_OFFSET      0x0138 /* Endpoint 2 Status Register */
#define AVR32_USBB_UESTA3_OFFSET      0x013c /* Endpoint 3 Status Register */
#define AVR32_USBB_UESTA4_OFFSET      0x0140 /* Endpoint 4 Status Register */
#define AVR32_USBB_UESTA5_OFFSET      0x0144 /* Endpoint 5 Status Register */
#define AVR32_USBB_UESTA6_OFFSET      0x0148 /* Endpoint 6 Status Register */
#define AVR32_USBB_UESTA7OFFSET       0x014c /* Endpoint 7 Status Register */

#define AVR32_USBB_UESTACLR_OFFSET(n) (0x0160+((n)<<2))
#define AVR32_USBB_UESTA0CLR_OFFSET   0x0160 /* Endpoint 0 Status Clear Register */
#define AVR32_USBB_UESTA1CLR_OFFSET   0x0164 /* Endpoint 1 Status Clear Register */
#define AVR32_USBB_UESTA2CLR_OFFSET   0x0168 /* Endpoint 2 Status Clear Register */
#define AVR32_USBB_UESTA3CLR_OFFSET   0x016c /* Endpoint 3 Status Clear Register */
#define AVR32_USBB_UESTA4CLR_OFFSET   0x0170 /* Endpoint 4 Status Clear Register */
#define AVR32_USBB_UESTA5CLR_OFFSET   0x0174 /* Endpoint 5 Status Clear Register */
#define AVR32_USBB_UESTA6CLR_OFFSET   0x0178 /* Endpoint 6 Status Clear Register */
#define AVR32_USBB_UESTA7CLR_OFFSET   0x017c /* Endpoint 7 Status Clear Register */

#define AVR32_USBB_UESTASET_OFFSET(n) (0x0190+((n)<<2))
#define AVR32_USBB_UESTA0SET_OFFSET   0x0190 /* Endpoint 0 Status Set Register */
#define AVR32_USBB_UESTA1SET_OFFSET   0x0194 /* Endpoint 1 Status Set Register */
#define AVR32_USBB_UESTA2SET_OFFSET   0x0198 /* Endpoint 2 Status Set Register */
#define AVR32_USBB_UESTA3SET_OFFSET   0x019c /* Endpoint 3 Status Set Register */
#define AVR32_USBB_UESTA4SET_OFFSET   0x01a0 /* Endpoint 4 Status Set Register */
#define AVR32_USBB_UESTA5SET_OFFSET   0x01a4 /* Endpoint 5 Status Set Register */
#define AVR32_USBB_UESTA6SET_OFFSET   0x01a8 /* Endpoint 6 Status Set Register */
#define AVR32_USBB_UESTA7SET_OFFSET   0x01ac /* Endpoint 7 Status Set Register */

#define AVR32_USBB_UECON_OFFSET(n)    (0x01c0+((n)<<2))
#define AVR32_USBB_UECON0_OFFSET      0x01c0 /* Endpoint 0 Control Register */
#define AVR32_USBB_UECON1_OFFSET      0x01c4 /* Endpoint 1 Control Register */
#define AVR32_USBB_UECON2_OFFSET      0x01c8 /* Endpoint 2 Control Register */
#define AVR32_USBB_UECON3_OFFSET      0x01cc /* Endpoint 3 Control Register */
#define AVR32_USBB_UECON4_OFFSET      0x01d0 /* Endpoint 4 Control Register */
#define AVR32_USBB_UECON5_OFFSET      0x01d4 /* Endpoint 5 Control Register */
#define AVR32_USBB_UECON6_OFFSET      0x01d8 /* Endpoint 7 Control Register */
#define AVR32_USBB_UECON7_OFFSET      0x01dc /* Endpoint 7 Control Register */

#define AVR32_USBB_UECONSET_OFFSET(n) (0x01f0+((n)<<2))
#define AVR32_USBB_UECON0SET_OFFSET   0x01f0 /* Endpoint 0 Control Set Register */
#define AVR32_USBB_UECON1SET_OFFSET   0x01f4 /* Endpoint 1 Control Set Register */
#define AVR32_USBB_UECON2SET_OFFSET   0x01f8 /* Endpoint 2 Control Set Register */
#define AVR32_USBB_UECON3SET_OFFSET   0x01fc /* Endpoint 3 Control Set Register */
#define AVR32_USBB_UECON4SET_OFFSET   0x0200 /* Endpoint 4 Control Set Register */
#define AVR32_USBB_UECON5SET_OFFSET   0x0204 /* Endpoint 5 Control Set Register */
#define AVR32_USBB_UECON6SET_OFFSET   0x0208 /* Endpoint 6 Control Set Register */
#define AVR32_USBB_UECON7SET_OFFSET   0x020c /* Endpoint 7 Control Set Register */

#define AVR32_USBB_UECONCLR_OFFSET(n) (0x0220+((n)<<2))
#define AVR32_USBB_UECON0CLR_OFFSET   0x0220 /* Endpoint 0 Control Clear Register */
#define AVR32_USBB_UECON1CLR_OFFSET   0x0224 /* Endpoint 1 Control Clear Register */
#define AVR32_USBB_UECON2CLR_OFFSET   0x0228 /* Endpoint 2 Control Clear Register */
#define AVR32_USBB_UECON3CLR_OFFSET   0x022c /* Endpoint 3 Control Clear Register */
#define AVR32_USBB_UECON4CLR_OFFSET   0x0230 /* Endpoint 4 Control Clear Register */
#define AVR32_USBB_UECON5CLR_OFFSET   0x0234 /* Endpoint 5 Control Clear Register */
#define AVR32_USBB_UECON6CLR_OFFSET   0x0238 /* Endpoint 6 Control Clear Register */
#define AVR32_USBB_UECON7CLR_OFFSET   0x023c /* Endpoint 7 Control Clear Register */

#define AVR32_UDDMA1_NEXTDESC_OFFSET  0x0310 /* Device DMA Channel 1 Next Descriptor Address Register */
#define AVR32_UDDMA1_ADDR_OFFSET      0x0314 /* Device DMA Channel 1 HSB Address Register */
#define AVR32_UDDMA1_CONTROL_OFFSET   0x0318 /* Device DMA Channel 1 Control Register */
#define AVR32_UDDMA1_STATUS_OFFSET    0x031c /* Device DMA Channel 1 Status Register */

#define AVR32_UDDMA2_NEXTDESC_OFFSET  0x0320 /* Device DMA Channel 2 Next Descriptor Address Register */
#define AVR32_UDDMA2_ADDR_OFFSET      0x0324 /* Device DMA Channel 2 HSB Address Register */
#define AVR32_UDDMA2_CONTROL_OFFSET   0x0328 /* Device DMA Channel 2 Control Register */
#define AVR32_UDDMA2_STATUS_OFFSET    0x032c /* Device DMA Channel 2 Status Register */

#define AVR32_UDDMA3_NEXTDESC_OFFSET  0x0330 /* Device DMA Channel 3 Next Descriptor Address Register */
#define AVR32_UDDMA3_ADDR_OFFSET      0x0334 /* Device DMA Channel 3 HSB Address Register */
#define AVR32_UDDMA3_CONTROL_OFFSET   0x0338 /* Device DMA Channel 3 Control Register */
#define AVR32_UDDMA3_STATUS_OFFSET    0x033c /* Device DMA Channel 3 Status Register */

#define AVR32_UDDMA4_NEXTDESC_OFFSET  0x0340 /* Device DMA Channel 4 Next Descriptor Address Register */
#define AVR32_UDDMA4_ADDR_OFFSET      0x0344 /* Device DMA Channel 4 HSB Address Register */
#define AVR32_UDDMA4_CONTROL_OFFSET   0x0348 /* Device DMA Channel 4 Control Register */
#define AVR32_UDDMA4_STATUS_OFFSET    0x034c /* Device DMA Channel 4 Status Register */

#define AVR32_UDDMA5_NEXTDESC_OFFSET  0x0350 /* Device DMA Channel 5 Next Descriptor Address Register */
#define AVR32_UDDMA5_ADDR_OFFSET      0x0354 /* Device DMA Channel 5 HSB Address Register */
#define AVR32_UDDMA5_CONTROL_OFFSET   0x0358 /* Device DMA Channel 5 Control Register */
#define AVR32_UDDMA5_STATUS_OFFSET    0x035c /* Device DMA Channel 5 Status Register */

#define AVR32_UDDMA6_NEXTDESC_OFFSET  0x0360 /* Device DMA Channel 6 Next Descriptor Address Register */
#define AVR32_UDDMA6_ADDR_OFFSET      0x0364 /* Device DMA Channel 6 HSB Address Register */
#define AVR32_UDDMA6_CONTROL_OFFSET   0x0368 /* Device DMA Channel 6 Control Register */
#define AVR32_UDDMA6_STATUS_OFFSET    0x036c /* Device DMA Channel 6 Status Register */

#define AVR32_USBB_UHCON_OFFSET       0x0400 /* Host General Control Register */
#define AVR32_USBB_UHINT_OFFSET       0x0404 /* Host Global Interrupt Register */
#define AVR32_USBB_UHINTCLR_OFFSET    0x0408 /* Host Global Interrupt Clear Register */
#define AVR32_USBB_UHINTSET_OFFSET    0x040c /* Host Global Interrupt Set Register */
#define AVR32_USBB_UHINTE_OFFSET      0x0410 /* Host Global Interrupt Enable Register */
#define AVR32_USBB_UHINTECLR_OFFSET   0x0414 /* Host Global Interrupt Enable Clear Register */
#define AVR32_USBB_UHINTESET_OFFSET   0x0418 /* Host Global Interrupt Enable Set Register */
#define AVR32_USBB_UPRST_OFFSET       0x041c /* Pipe Enable/Reset Register */
#define AVR32_USBB_UHFNUM_OFFSET      0x0420 /* Host Frame Number Register */
#define AVR32_USBB_UHADDR1_OFFSET     0x0424 /* Host Address 1 Register */
#define AVR32_USBB_UHADDR2_OFFSET     0x0428 /* Host Address 2 Register */

#define AVR32_USBB_UPCFG_OFFSET(n)    (0x0500+((n)<<2))
#define AVR32_USBB_UPCFG0_OFFSET      0x0500 /* Pipe 0 Configuration Register */
#define AVR32_USBB_UPCFG1_OFFSET      0x0504 /* Pipe 1 Configuration Register */
#define AVR32_USBB_UPCFG2_OFFSET      0x0508 /* Pipe 2 Configuration Register */
#define AVR32_USBB_UPCFG3_OFFSET      0x050c /* Pipe 3 Configuration Register */
#define AVR32_USBB_UPCFG4_OFFSET      0x0510 /* Pipe 4 Configuration Register */
#define AVR32_USBB_UPCFG5_OFFSET      0x0514 /* Pipe 5 Configuration Register */
#define AVR32_USBB_UPCFG6_OFFSET      0x0518 /* Pipe 6 Configuration Register */
#define AVR32_USBB_UPCFG7_OFFSET      0x051c /* Pipe 7 Configuration Register */

#define AVR32_USBB_UPSTA_OFFSET(n)    (0x0530+((n)<<2))
#define AVR32_USBB_UPSTA0_OFFSET      0x0530 /* Pipe 0 Status Register */
#define AVR32_USBB_UPSTA1_OFFSET      0x0534 /* Pipe 1 Status Register */
#define AVR32_USBB_UPSTA2_OFFSET      0x0538 /* Pipe 2 Status Register */
#define AVR32_USBB_UPSTA3_OFFSET      0x053c /* Pipe 3 Status Register */
#define AVR32_USBB_UPSTA4_OFFSET      0x0540 /* Pipe 4 Status Register */
#define AVR32_USBB_UPSTA5_OFFSET      0x0544 /* Pipe 5 Status Register */
#define AVR32_USBB_UPSTA6_OFFSET      0x0548 /* Pipe 6 Status Register */
#define AVR32_USBB_UPSTA7_OFFSET      0x054c /* Pipe 7 Status Register */

#define AVR32_USBB_UPSTACLR_OFFSET(n) (0x0560+((n)<<2))
#define AVR32_USBB_UPSTA0CLR_OFFSET   0x0560 /* Pipe 0 Status Clear Register */
#define AVR32_USBB_UPSTA1CLR_OFFSET   0x0564 /* Pipe 1 Status Clear Register */
#define AVR32_USBB_UPSTA2CLR_OFFSET   0x0568 /* Pipe 2 Status Clear Register */
#define AVR32_USBB_UPSTA3CLR_OFFSET   0x056c /* Pipe 3 Status Clear Register */
#define AVR32_USBB_UPSTA4CLR_OFFSET   0x0570 /* Pipe 4 Status Clear Register */
#define AVR32_USBB_UPSTA5CLR_OFFSET   0x0574 /* Pipe 5 Status Clear Register */
#define AVR32_USBB_UPSTA6CLR_OFFSET   0x0578 /* Pipe 6 Status Clear Register */
#define AVR32_USBB_UPSTA7CLR_OFFSET   0x057c /* Pipe 7 Status Clear Register */

#define AVR32_USBB_UPSTASET_OFFSET(n) (0x0590+((n)<<2))
#define AVR32_USBB_UPSTA0SET_OFFSET   0x0590 /* Pipe 0 Status Set Register */
#define AVR32_USBB_UPSTA1SET_OFFSET   0x0594 /* Pipe 1 Status Set Register */
#define AVR32_USBB_UPSTA2SET_OFFSET   0x0598 /* Pipe 2 Status Set Register */
#define AVR32_USBB_UPSTA3SET_OFFSET   0x059c /* Pipe 3 Status Set Register */
#define AVR32_USBB_UPSTA4SET_OFFSET   0x05a0 /* Pipe 4 Status Set Register */
#define AVR32_USBB_UPSTA5SET_OFFSET   0x05a4 /* Pipe 5 Status Set Register */
#define AVR32_USBB_UPSTA6SET_OFFSET   0x05a8 /* Pipe 6 Status Set Register */
#define AVR32_USBB_UPSTA7SET_OFFSET   0x05ac /* Pipe 7 Status Set Register */

#define AVR32_USBB_UPCON_OFFSET(n)    (0x05c0+((n)<<2))
#define AVR32_USBB_UPCON0_OFFSET      0x05c0 /* Pipe 0 Control Register */
#define AVR32_USBB_UPCON1_OFFSET      0x05c4 /* Pipe 1 Control Register */
#define AVR32_USBB_UPCON2_OFFSET      0x05c8 /* Pipe 2 Control Register */
#define AVR32_USBB_UPCON3_OFFSET      0x05cc /* Pipe 3 Control Register */
#define AVR32_USBB_UPCON4_OFFSET      0x05d0 /* Pipe 4 Control Register */
#define AVR32_USBB_UPCON5_OFFSET      0x05d4 /* Pipe 5 Control Register */
#define AVR32_USBB_UPCON6_OFFSET      0x05d8 /* Pipe 6 Control Register */
#define AVR32_USBB_UPCON7_OFFSET      0x05dc /* Pipe 7 Control Register */

#define AVR32_USBB_UPCONSET_OFFSET(n) (0x05f0+((n)<<2))
#define AVR32_USBB_UPCON0SET_OFFSET   0x05f0 /* Pipe 0 Control Set Register */
#define AVR32_USBB_UPCON1SET_OFFSET   0x05f4 /* Pipe 1 Control Set Register */
#define AVR32_USBB_UPCON2SET_OFFSET   0x05f8 /* Pipe 2 Control Set Register */
#define AVR32_USBB_UPCON3SET_OFFSET   0x05fc /* Pipe 3 Control Set Register */
#define AVR32_USBB_UPCON4SET_OFFSET   0x0600 /* Pipe 4 Control Set Register */
#define AVR32_USBB_UPCON5SET_OFFSET   0x0604 /* Pipe 5 Control Set Register */
#define AVR32_USBB_UPCON6SET_OFFSET   0x0608 /* Pipe 6 Control Set Register */
#define AVR32_USBB_UPCON7SET_OFFSET   0x060c /* Pipe 7 Control Set Register */

#define AVR32_USBB_UPCONCLR_OFFSET(n) (0x0620+((n)<<2))
#define AVR32_USBB_UPCON0CLR_OFFSET   0x0620 /* Pipe 0 Control Clear Register */
#define AVR32_USBB_UPCON1CLR_OFFSET   0x0624 /* Pipe 1 Control Clear Register */
#define AVR32_USBB_UPCON2CLR_OFFSET   0x0628 /* Pipe 2 Control Clear Register */
#define AVR32_USBB_UPCON3CLR_OFFSET   0x062c /* Pipe 3 Control Clear Register */
#define AVR32_USBB_UPCON4CLR_OFFSET   0x0630 /* Pipe 4 Control Clear Register */
#define AVR32_USBB_UPCON5CLR_OFFSET   0x0634 /* Pipe 5 Control Clear Register */
#define AVR32_USBB_UPCON6CLR_OFFSET   0x0638 /* Pipe 6 Control Clear Register */
#define AVR32_USBB_UPCON7CLR_OFFSET   0x063c /* Pipe 7 Control Clear Register */

#define AVR32_USBB_UPINRQ_OFFSET(n)   (0x0650+((n)<<2))
#define AVR32_USBB_UPINRQ0_OFFSET     0x0650 /* Pipe 0 IN Request Register */
#define AVR32_USBB_UPINRQ1_OFFSET     0x0654 /* Pipe 1 IN Request Register */
#define AVR32_USBB_UPINRQ2_OFFSET     0x0658 /* Pipe 2 IN Request Register */
#define AVR32_USBB_UPINRQ3_OFFSET     0x065c /* Pipe 3 IN Request Register */
#define AVR32_USBB_UPINRQ4_OFFSET     0x0660 /* Pipe 4 IN Request Register */
#define AVR32_USBB_UPINRQ5_OFFSET     0x0664 /* Pipe 5 IN Request Register */
#define AVR32_USBB_UPINRQ6_OFFSET     0x0668 /* Pipe 6 IN Request Register */
#define AVR32_USBB_UPINRQ7_OFFSET     0x066c /* Pipe 7 IN Request Register */

#define AVR32_USBB_UPERR_OFFSET(n)    (0x0680+((n)<<2))
#define AVR32_USBB_UPERR0_OFFSET      0x0680 /* Pipe 0 Error Register */
#define AVR32_USBB_UPERR1_OFFSET      0x0684 /* Pipe 1 Error Register */
#define AVR32_USBB_UPERR2_OFFSET      0x0688 /* Pipe 2 Error Register */
#define AVR32_USBB_UPERR3_OFFSET      0x068c /* Pipe 3 Error Register */
#define AVR32_USBB_UPERR4_OFFSET      0x0690 /* Pipe 4 Error Register */
#define AVR32_USBB_UPERR5_OFFSET      0x0694 /* Pipe 5 Error Register */
#define AVR32_USBB_UPERR6_OFFSET      0x0698 /* Pipe 6 Error Register */
#define AVR32_USBB_UPERR7_OFFSET      0x069c /* Pipe 7 Error Register */

#define AVR32_UHDMA1_NEXTDESC_OFFSET  0x0710 /* Host DMA Channel 1 Next Descriptor Address Register */
#define AVR32_UHDMA1_ADDR_OFFSET      0x0714 /* Host DMA Channel 1 HSB Address Register */
#define AVR32_UHDMA1_CONTROL_OFFSET   0x0718 /* Host DMA Channel 1 Control Register */
#define AVR32_UHDMA1_STATUS_OFFSET    0x071c /* Host DMA Channel 1 Status Register */

#define AVR32_UHDMA2_NEXTDESC_OFFSET  0x0720 /* Host DMA Channel 2 Next Descriptor Address Register */
#define AVR32_UHDMA2_ADDR_OFFSET      0x0724 /* Host DMA Channel 2 HSB Address Register */
#define AVR32_UHDMA2_CONTROL_OFFSET   0x0728 /* Host DMA Channel 2 Control Register */
#define AVR32_UHDMA2_STATUS_OFFSET    0x072c /* Host DMA Channel 2 Status Register */

#define AVR32_UHDMA3_NEXTDESC_OFFSET  0x0730 /* Host DMA Channel 3 Next Descriptor Address Register */
#define AVR32_UHDMA3_ADDR_OFFSET      0x0734 /* Host DMA Channel 3 HSB Address Register */
#define AVR32_UHDMA3_CONTROL_OFFSET   0x0738 /* Host DMA Channel 3 Control Register */
#define AVR32_UHDMA3_STATUS_OFFSET    0x073c /* Host DMA Channel 3 Status Register */

#define AVR32_UHDMA4_NEXTDESC_OFFSET  0x0740 /* Host DMA Channel 4 Next Descriptor Address Register */
#define AVR32_UHDMA4_ADDR_OFFSET      0x0744 /* Host DMA Channel 4 HSB Address Register */
#define AVR32_UHDMA4_CONTROL_OFFSET   0x0748 /* Host DMA Channel 4 Control Register */
#define AVR32_UHDMA4_STATUS_OFFSET    0x074c /* Host DMA Channel 4 Status Register */

#define AVR32_UHDMA5_NEXTDESC_OFFSET  0x0750 /* Host DMA Channel 5 Next Descriptor Address Register */
#define AVR32_UHDMA5_ADDR_OFFSET      0x0754 /* Host DMA Channel 5 HSB Address Register */
#define AVR32_UHDMA5_CONTROL_OFFSET   0x0758 /* Host DMA Channel 5 Control Register */
#define AVR32_UHDMA5_STATUS_OFFSET    0x075c /* Host DMA Channel 5 Status Register */

#define AVR32_UHDMA6_NEXTDESC_OFFSET  0x0760 /* Host DMA Channel 6 Next Descriptor Address Register */
#define AVR32_UHDMA6_ADDR_OFFSET      0x0764 /* Host DMA Channel 6 HSB Address Register */
#define AVR32_UHDMA6_CONTROL_OFFSET   0x0768 /* Host DMA Channel 6 Control Register */
#define AVR32_UHDMA6_STATUS_OFFSET    0x076c /* Host DMA Channel 6 Status Register */

#define AVR32_USBB_USBCON_OFFSET      0x0800 /* General Control Register */
#define AVR32_USBB_USBSTA_OFFSET      0x0804 /* General Status Register */
#define AVR32_USBB_USBSTACLR_OFFSET   0x0808 /* General Status Clear Register */
#define AVR32_USBB_USBSTASET_OFFSET   0x080c /* General Status Set Register */
#define AVR32_USBB_UVERS_OFFSET       0x0818 /* IP Version Register */
#define AVR32_USBB_UFEATURES_OFFSET   0x081c /* IP Features Register */
#define AVR32_USBB_UADDRSIZE_OFFSET   0x0820 /* IP PB Address Size Register */
#define AVR32_USBB_UNAME1_OFFSET      0x0824 /* IP Name Register 1 */
#define AVR32_USBB_UNAME2_OFFSET      0x0828 /* IP Name Register 2 */
#define AVR32_USBB_USBFSM_OFFSET      0x082c /* USB Finite State Machine Status Register */

/* Register Addresses ***************************************************************/

#warning "Missing Logic"

/* Register Bit-field Definitions ***************************************************/

#warning "Missing Logic"

/* USB HSB Memory Map ***************************************************************/

#define USB_FIFO0_DATA_OFFSET         0x00000 /* Pipe/Endpoint 0 FIFO Data Register */
#define USB_FIFO1_DATA_OFFSET         0x10000 /* Pipe/Endpoint 1 FIFO Data Register */
#define USB_FIFO2_DATA_OFFSET         0x20000 /* Pipe/Endpoint 2 FIFO Data Register */
#define USB_FIFO3_DATA_OFFSET         0x30000 /* Pipe/Endpoint 3 FIFO Data Register */
#define USB_FIFO4_DATA_OFFSET         0x40000 /* Pipe/Endpoint 4 FIFO Data Register */
#define USB_FIFO5_DATA_OFFSET         0x50000 /* Pipe/Endpoint 5 FIFO Data Register */
#define USB_FIFO6_DATA_OFFSET         0x60000 /* Pipe/Endpoint 6 FIFO Data Register */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT91UC3_AT91UC3_USBB_H */


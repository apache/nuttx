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

#define AVR32_USBB_UESTA_OFFSET(n)    (0x0130+((n)<<2))
#define AVR32_USBB_UESTA0_OFFSET      0x0130 /* Endpoint 0 Status Register */
#define AVR32_USBB_UESTA1_OFFSET      0x0134 /* Endpoint 1 Status Register */
#define AVR32_USBB_UESTA2_OFFSET      0x0138 /* Endpoint 2 Status Register */
#define AVR32_USBB_UESTA3_OFFSET      0x013c /* Endpoint 3 Status Register */
#define AVR32_USBB_UESTA4_OFFSET      0x0140 /* Endpoint 4 Status Register */
#define AVR32_USBB_UESTA5_OFFSET      0x0144 /* Endpoint 5 Status Register */
#define AVR32_USBB_UESTA6_OFFSET      0x0148 /* Endpoint 6 Status Register */

#define AVR32_USBB_UESTACLR_OFFSET(n) (0x0160+((n)<<2))
#define AVR32_USBB_UESTA0CLR_OFFSET   0x0160 /* Endpoint 0 Status Clear Register */
#define AVR32_USBB_UESTA1CLR_OFFSET   0x0164 /* Endpoint 1 Status Clear Register */
#define AVR32_USBB_UESTA2CLR_OFFSET   0x0168 /* Endpoint 2 Status Clear Register */
#define AVR32_USBB_UESTA3CLR_OFFSET   0x016c /* Endpoint 3 Status Clear Register */
#define AVR32_USBB_UESTA4CLR_OFFSET   0x0170 /* Endpoint 4 Status Clear Register */
#define AVR32_USBB_UESTA5CLR_OFFSET   0x0174 /* Endpoint 5 Status Clear Register */
#define AVR32_USBB_UESTA6CLR_OFFSET   0x0178 /* Endpoint 6 Status Clear Register */

#define AVR32_USBB_UESTASET_OFFSET(n) (0x0190+((n)<<2))
#define AVR32_USBB_UESTA0SET_OFFSET   0x0190 /* Endpoint 0 Status Set Register */
#define AVR32_USBB_UESTA1SET_OFFSET   0x0194 /* Endpoint 1 Status Set Register */
#define AVR32_USBB_UESTA2SET_OFFSET   0x0198 /* Endpoint 2 Status Set Register */
#define AVR32_USBB_UESTA3SET_OFFSET   0x019c /* Endpoint 3 Status Set Register */
#define AVR32_USBB_UESTA4SET_OFFSET   0x01a0 /* Endpoint 4 Status Set Register */
#define AVR32_USBB_UESTA5SET_OFFSET   0x01a4 /* Endpoint 5 Status Set Register */
#define AVR32_USBB_UESTA6SET_OFFSET   0x01a8 /* Endpoint 6 Status Set Register */

#define AVR32_USBB_UECON_OFFSET(n)    (0x01c0+((n)<<2))
#define AVR32_USBB_UECON0_OFFSET      0x01c0 /* Endpoint 0 Control Register */
#define AVR32_USBB_UECON1_OFFSET      0x01c4 /* Endpoint 1 Control Register */
#define AVR32_USBB_UECON2_OFFSET      0x01c8 /* Endpoint 2 Control Register */
#define AVR32_USBB_UECON3_OFFSET      0x01cc /* Endpoint 3 Control Register */
#define AVR32_USBB_UECON4_OFFSET      0x01d0 /* Endpoint 4 Control Register */
#define AVR32_USBB_UECON5_OFFSET      0x01d4 /* Endpoint 5 Control Register */
#define AVR32_USBB_UECON6_OFFSET      0x01d8 /* Endpoint 7 Control Register */

#define AVR32_USBB_UECONSET_OFFSET(n) (0x01f0+((n)<<2))
#define AVR32_USBB_UECON0SET_OFFSET   0x01f0 /* Endpoint 0 Control Set Register */
#define AVR32_USBB_UECON1SET_OFFSET   0x01f4 /* Endpoint 1 Control Set Register */
#define AVR32_USBB_UECON2SET_OFFSET   0x01f8 /* Endpoint 2 Control Set Register */
#define AVR32_USBB_UECON3SET_OFFSET   0x01fc /* Endpoint 3 Control Set Register */
#define AVR32_USBB_UECON4SET_OFFSET   0x0200 /* Endpoint 4 Control Set Register */
#define AVR32_USBB_UECON5SET_OFFSET   0x0204 /* Endpoint 5 Control Set Register */
#define AVR32_USBB_UECON6SET_OFFSET   0x0208 /* Endpoint 6 Control Set Register */

#define AVR32_USBB_UECONCLR_OFFSET(n) (0x0220+((n)<<2))
#define AVR32_USBB_UECON0CLR_OFFSET   0x0220 /* Endpoint 0 Control Clear Register */
#define AVR32_USBB_UECON1CLR_OFFSET   0x0224 /* Endpoint 1 Control Clear Register */
#define AVR32_USBB_UECON2CLR_OFFSET   0x0228 /* Endpoint 2 Control Clear Register */
#define AVR32_USBB_UECON3CLR_OFFSET   0x022c /* Endpoint 3 Control Clear Register */
#define AVR32_USBB_UECON4CLR_OFFSET   0x0230 /* Endpoint 4 Control Clear Register */
#define AVR32_USBB_UECON5CLR_OFFSET   0x0234 /* Endpoint 5 Control Clear Register */
#define AVR32_USBB_UECON6CLR_OFFSET   0x0238 /* Endpoint 6 Control Clear Register */

#define AVR32_UDDMA_OFFSET(n)         (0x0300+((n)<<4))
#define AVR32_UDDMA_NEXTDESC_OFFSET   0x0000 /* Device DMA Channel Next Descriptor Address Register */
#define AVR32_UDDMA_ADDR_OFFSET       0x0004 /* Device DMA Channel HSB Address Register */
#define AVR32_UDDMA_CONTROL_OFFSET    0x0008 /* Device DMA Channel Control Register */
#define AVR32_UDDMA_STATUS_OFFSET     0x000c /* Device DMA Channel Status Register */

#define AVR32_UDDMA1_NEXTDESC_OFFSET  0x0310 /* Device DMA Channel 1 Next Descriptor Address Register */
#define AVR32_UDDMA1_ADDR_OFFSET      0x0314 /* Device DMA Channel 1 HSB Address Register */
#define AVR32_UDDMA1_CONTROL_OFFSET   0x0318 /* Device DMA Channel 1 Control Register */
#define AVR32_UDDMA1_STATUS_OFFSET    0x031c /* Device DMA Channel 1 Status Register */

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

#define AVR32_USBB_UPSTA_OFFSET(n)    (0x0530+((n)<<2))
#define AVR32_USBB_UPSTA0_OFFSET      0x0530 /* Pipe 0 Status Register */
#define AVR32_USBB_UPSTA1_OFFSET      0x0534 /* Pipe 1 Status Register */
#define AVR32_USBB_UPSTA2_OFFSET      0x0538 /* Pipe 2 Status Register */
#define AVR32_USBB_UPSTA3_OFFSET      0x053c /* Pipe 3 Status Register */
#define AVR32_USBB_UPSTA4_OFFSET      0x0540 /* Pipe 4 Status Register */
#define AVR32_USBB_UPSTA5_OFFSET      0x0544 /* Pipe 5 Status Register */
#define AVR32_USBB_UPSTA6_OFFSET      0x0548 /* Pipe 6 Status Register */

#define AVR32_USBB_UPSTACLR_OFFSET(n) (0x0560+((n)<<2))
#define AVR32_USBB_UPSTA0CLR_OFFSET   0x0560 /* Pipe 0 Status Clear Register */
#define AVR32_USBB_UPSTA1CLR_OFFSET   0x0564 /* Pipe 1 Status Clear Register */
#define AVR32_USBB_UPSTA2CLR_OFFSET   0x0568 /* Pipe 2 Status Clear Register */
#define AVR32_USBB_UPSTA3CLR_OFFSET   0x056c /* Pipe 3 Status Clear Register */
#define AVR32_USBB_UPSTA4CLR_OFFSET   0x0570 /* Pipe 4 Status Clear Register */
#define AVR32_USBB_UPSTA5CLR_OFFSET   0x0574 /* Pipe 5 Status Clear Register */
#define AVR32_USBB_UPSTA6CLR_OFFSET   0x0578 /* Pipe 6 Status Clear Register */

#define AVR32_USBB_UPSTASET_OFFSET(n) (0x0590+((n)<<2))
#define AVR32_USBB_UPSTA0SET_OFFSET   0x0590 /* Pipe 0 Status Set Register */
#define AVR32_USBB_UPSTA1SET_OFFSET   0x0594 /* Pipe 1 Status Set Register */
#define AVR32_USBB_UPSTA2SET_OFFSET   0x0598 /* Pipe 2 Status Set Register */
#define AVR32_USBB_UPSTA3SET_OFFSET   0x059c /* Pipe 3 Status Set Register */
#define AVR32_USBB_UPSTA4SET_OFFSET   0x05a0 /* Pipe 4 Status Set Register */
#define AVR32_USBB_UPSTA5SET_OFFSET   0x05a4 /* Pipe 5 Status Set Register */
#define AVR32_USBB_UPSTA6SET_OFFSET   0x05a8 /* Pipe 6 Status Set Register */

#define AVR32_USBB_UPCON_OFFSET(n)    (0x05c0+((n)<<2))
#define AVR32_USBB_UPCON0_OFFSET      0x05c0 /* Pipe 0 Control Register */
#define AVR32_USBB_UPCON1_OFFSET      0x05c4 /* Pipe 1 Control Register */
#define AVR32_USBB_UPCON2_OFFSET      0x05c8 /* Pipe 2 Control Register */
#define AVR32_USBB_UPCON3_OFFSET      0x05cc /* Pipe 3 Control Register */
#define AVR32_USBB_UPCON4_OFFSET      0x05d0 /* Pipe 4 Control Register */
#define AVR32_USBB_UPCON5_OFFSET      0x05d4 /* Pipe 5 Control Register */
#define AVR32_USBB_UPCON6_OFFSET      0x05d8 /* Pipe 6 Control Register */

#define AVR32_USBB_UPCONSET_OFFSET(n) (0x05f0+((n)<<2))
#define AVR32_USBB_UPCON0SET_OFFSET   0x05f0 /* Pipe 0 Control Set Register */
#define AVR32_USBB_UPCON1SET_OFFSET   0x05f4 /* Pipe 1 Control Set Register */
#define AVR32_USBB_UPCON2SET_OFFSET   0x05f8 /* Pipe 2 Control Set Register */
#define AVR32_USBB_UPCON3SET_OFFSET   0x05fc /* Pipe 3 Control Set Register */
#define AVR32_USBB_UPCON4SET_OFFSET   0x0600 /* Pipe 4 Control Set Register */
#define AVR32_USBB_UPCON5SET_OFFSET   0x0604 /* Pipe 5 Control Set Register */
#define AVR32_USBB_UPCON6SET_OFFSET   0x0608 /* Pipe 6 Control Set Register */

#define AVR32_USBB_UPCONCLR_OFFSET(n) (0x0620+((n)<<2))
#define AVR32_USBB_UPCON0CLR_OFFSET   0x0620 /* Pipe 0 Control Clear Register */
#define AVR32_USBB_UPCON1CLR_OFFSET   0x0624 /* Pipe 1 Control Clear Register */
#define AVR32_USBB_UPCON2CLR_OFFSET   0x0628 /* Pipe 2 Control Clear Register */
#define AVR32_USBB_UPCON3CLR_OFFSET   0x062c /* Pipe 3 Control Clear Register */
#define AVR32_USBB_UPCON4CLR_OFFSET   0x0630 /* Pipe 4 Control Clear Register */
#define AVR32_USBB_UPCON5CLR_OFFSET   0x0634 /* Pipe 5 Control Clear Register */
#define AVR32_USBB_UPCON6CLR_OFFSET   0x0638 /* Pipe 6 Control Clear Register */

#define AVR32_USBB_UPINRQ_OFFSET(n)   (0x0650+((n)<<2))
#define AVR32_USBB_UPINRQ0_OFFSET     0x0650 /* Pipe 0 IN Request Register */
#define AVR32_USBB_UPINRQ1_OFFSET     0x0654 /* Pipe 1 IN Request Register */
#define AVR32_USBB_UPINRQ2_OFFSET     0x0658 /* Pipe 2 IN Request Register */
#define AVR32_USBB_UPINRQ3_OFFSET     0x065c /* Pipe 3 IN Request Register */
#define AVR32_USBB_UPINRQ4_OFFSET     0x0660 /* Pipe 4 IN Request Register */
#define AVR32_USBB_UPINRQ5_OFFSET     0x0664 /* Pipe 5 IN Request Register */
#define AVR32_USBB_UPINRQ6_OFFSET     0x0668 /* Pipe 6 IN Request Register */

#define AVR32_USBB_UPERR_OFFSET(n)    (0x0680+((n)<<2))
#define AVR32_USBB_UPERR0_OFFSET      0x0680 /* Pipe 0 Error Register */
#define AVR32_USBB_UPERR1_OFFSET      0x0684 /* Pipe 1 Error Register */
#define AVR32_USBB_UPERR2_OFFSET      0x0688 /* Pipe 2 Error Register */
#define AVR32_USBB_UPERR3_OFFSET      0x068c /* Pipe 3 Error Register */
#define AVR32_USBB_UPERR4_OFFSET      0x0690 /* Pipe 4 Error Register */
#define AVR32_USBB_UPERR5_OFFSET      0x0694 /* Pipe 5 Error Register */
#define AVR32_USBB_UPERR6_OFFSET      0x0698 /* Pipe 6 Error Register */

#define AVR32_UHDMA_OFFSET(n)         (0x0700+((n)<<4))
#define AVR32_UHDMA_NEXTDESC_OFFSET   0x0000 /* Host DMA Channel Next Descriptor Address Register */
#define AVR32_UHDMA_ADDR_OFFSET       0x0004 /* Host DMA Channel HSB Address Register */
#define AVR32_UHDMA_CONTROL_OFFSET    0x0008 /* Host DMA Channel Control Register */
#define AVR32_UHDMA_STATUS_OFFSET     0x000c /* Host DMA Channel Status Register */

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

#define AVR32_USBB_UDCON              (AVR32_USB_BASE+AVR32_USBB_UDCON_OFFSET)
#define AVR32_USBB_UDINT              (AVR32_USB_BASE+AVR32_USBB_UDINT_OFFSET)
#define AVR32_USBB_UDINTCLR           (AVR32_USB_BASE+AVR32_USBB_UDINTCLR_OFFSET)
#define AVR32_USBB_UDINTSET           (AVR32_USB_BASE+AVR32_USBB_UDINTSET_OFFSET)
#define AVR32_USBB_UDINTE             (AVR32_USB_BASE+AVR32_USBB_UDINTE_OFFSET)
#define AVR32_USBB_UDINTECLR          (AVR32_USB_BASE+AVR32_USBB_UDINTECLR_OFFSET)
#define AVR32_USBB_UDINTESET          (AVR32_USB_BASE+AVR32_USBB_UDINTESET_OFFSET)
#define AVR32_USBB_UERST              (AVR32_USB_BASE+AVR32_USBB_UERST_OFFSET)
#define AVR32_USBB_UDFNUM             (AVR32_USB_BASE+AVR32_USBB_UDFNUM_OFFSET)

#define AVR32_USBB_UECFG(n)           (AVR32_USB_BASE+AVR32_USBB_UECFG_OFFSET(n))
#define AVR32_USBB_UECFG0             (AVR32_USB_BASE+AVR32_USBB_UECFG0_OFFSET)
#define AVR32_USBB_UECFG1             (AVR32_USB_BASE+AVR32_USBB_UECFG1_OFFSET)
#define AVR32_USBB_UECFG2             (AVR32_USB_BASE+AVR32_USBB_UECFG2_OFFSET)
#define AVR32_USBB_UECFG3             (AVR32_USB_BASE+AVR32_USBB_UECFG3_OFFSET)
#define AVR32_USBB_UECFG4             (AVR32_USB_BASE+AVR32_USBB_UECFG4_OFFSET)
#define AVR32_USBB_UECFG5             (AVR32_USB_BASE+AVR32_USBB_UECFG5_OFFSET)
#define AVR32_USBB_UECFG6             (AVR32_USB_BASE+AVR32_USBB_UECFG6_OFFSET)

#define AVR32_USBB_UESTA(n)           (AVR32_USB_BASE+AVR32_USBB_UESTA_OFFSET(n))
#define AVR32_USBB_UESTA0             (AVR32_USB_BASE+AVR32_USBB_UESTA0_OFFSET)
#define AVR32_USBB_UESTA1             (AVR32_USB_BASE+AVR32_USBB_UESTA1_OFFSET)
#define AVR32_USBB_UESTA2             (AVR32_USB_BASE+AVR32_USBB_UESTA2_OFFSET)
#define AVR32_USBB_UESTA3             (AVR32_USB_BASE+AVR32_USBB_UESTA3_OFFSET)
#define AVR32_USBB_UESTA4             (AVR32_USB_BASE+AVR32_USBB_UESTA4_OFFSET)
#define AVR32_USBB_UESTA5             (AVR32_USB_BASE+AVR32_USBB_UESTA5_OFFSET)
#define AVR32_USBB_UESTA6             (AVR32_USB_BASE+AVR32_USBB_UESTA6_OFFSET)

#define AVR32_USBB_UESTACLR(n)        (AVR32_USB_BASE+AVR32_USBB_UESTACLR_OFFSET(n))
#define AVR32_USBB_UESTA0CLR          (AVR32_USB_BASE+AVR32_USBB_UESTA0CLR_OFFSET)
#define AVR32_USBB_UESTA1CLR          (AVR32_USB_BASE+AVR32_USBB_UESTA1CLR_OFFSET)
#define AVR32_USBB_UESTA2CLR          (AVR32_USB_BASE+AVR32_USBB_UESTA2CLR_OFFSET)
#define AVR32_USBB_UESTA3CLR          (AVR32_USB_BASE+AVR32_USBB_UESTA3CLR_OFFSET)
#define AVR32_USBB_UESTA4CLR          (AVR32_USB_BASE+AVR32_USBB_UESTA4CLR_OFFSET)
#define AVR32_USBB_UESTA5CLR          (AVR32_USB_BASE+AVR32_USBB_UESTA5CLR_OFFSET)
#define AVR32_USBB_UESTA6CLR          (AVR32_USB_BASE+AVR32_USBB_UESTA6CLR_OFFSET)

#define AVR32_USBB_UESTASET(n)        (AVR32_USB_BASE+AVR32_USBB_UESTASET_OFFSET(n))
#define AVR32_USBB_UESTA0SET          (AVR32_USB_BASE+AVR32_USBB_UESTA0SET_OFFSET)
#define AVR32_USBB_UESTA1SET          (AVR32_USB_BASE+AVR32_USBB_UESTA1SET_OFFSET)
#define AVR32_USBB_UESTA2SET          (AVR32_USB_BASE+AVR32_USBB_UESTA2SET_OFFSET)
#define AVR32_USBB_UESTA3SET          (AVR32_USB_BASE+AVR32_USBB_UESTA3SET_OFFSET)
#define AVR32_USBB_UESTA4SET          (AVR32_USB_BASE+AVR32_USBB_UESTA4SET_OFFSET)
#define AVR32_USBB_UESTA5SET          (AVR32_USB_BASE+AVR32_USBB_UESTA5SET_OFFSET)
#define AVR32_USBB_UESTA6SET          (AVR32_USB_BASE+AVR32_USBB_UESTA6SET_OFFSET)

#define AVR32_USBB_UECON(n)           (AVR32_USB_BASE+AVR32_USBB_UECON_OFFSET(n))
#define AVR32_USBB_UECON0             (AVR32_USB_BASE+AVR32_USBB_UECON0_OFFSET)
#define AVR32_USBB_UECON1             (AVR32_USB_BASE+AVR32_USBB_UECON1_OFFSET)
#define AVR32_USBB_UECON2             (AVR32_USB_BASE+AVR32_USBB_UECON2_OFFSET)
#define AVR32_USBB_UECON3             (AVR32_USB_BASE+AVR32_USBB_UECON3_OFFSET)
#define AVR32_USBB_UECON4             (AVR32_USB_BASE+AVR32_USBB_UECON4_OFFSET)
#define AVR32_USBB_UECON5             (AVR32_USB_BASE+AVR32_USBB_UECON5_OFFSET)
#define AVR32_USBB_UECON6             (AVR32_USB_BASE+AVR32_USBB_UECON6_OFFSET)

#define AVR32_USBB_UECONSET(n)        (AVR32_USB_BASE+AVR32_USBB_UECONSET_OFFSET(n))
#define AVR32_USBB_UECON0SET          (AVR32_USB_BASE+AVR32_USBB_UECON0SET_OFFSET)
#define AVR32_USBB_UECON1SET          (AVR32_USB_BASE+AVR32_USBB_UECON1SET_OFFSET)
#define AVR32_USBB_UECON2SET          (AVR32_USB_BASE+AVR32_USBB_UECON2SET_OFFSET)
#define AVR32_USBB_UECON3SET          (AVR32_USB_BASE+AVR32_USBB_UECON3SET_OFFSET)
#define AVR32_USBB_UECON4SET          (AVR32_USB_BASE+AVR32_USBB_UECON4SET_OFFSET)
#define AVR32_USBB_UECON5SET          (AVR32_USB_BASE+AVR32_USBB_UECON5SET_OFFSET)
#define AVR32_USBB_UECON6SET          (AVR32_USB_BASE+AVR32_USBB_UECON6SET_OFFSET)

#define AVR32_USBB_UECONCLR(n)        (AVR32_USB_BASE+AVR32_USBB_UECONCLR_OFFSET(n))
#define AVR32_USBB_UECON0CLR          (AVR32_USB_BASE+AVR32_USBB_UECON0CLR_OFFSET)
#define AVR32_USBB_UECON1CLR          (AVR32_USB_BASE+AVR32_USBB_UECON1CLR_OFFSET)
#define AVR32_USBB_UECON2CLR          (AVR32_USB_BASE+AVR32_USBB_UECON2CLR_OFFSET)
#define AVR32_USBB_UECON3CLR          (AVR32_USB_BASE+AVR32_USBB_UECON3CLR_OFFSET)
#define AVR32_USBB_UECON4CLR          (AVR32_USB_BASE+AVR32_USBB_UECON4CLR_OFFSET)
#define AVR32_USBB_UECON5CLR          (AVR32_USB_BASE+AVR32_USBB_UECON5CLR_OFFSET)
#define AVR32_USBB_UECON6CLR          (AVR32_USB_BASE+AVR32_USBB_UECON6CLR_OFFSET)

#define AVR32_UDDMA_BASE(n)           (AVR32_USB_BASE+AVR32_UDDMA_OFFSET(n))
#define AVR32_UDDMA_NEXTDESC(n)       (AVR32_UDDMA_BASE(n)+AVR32_UDDMA_NEXTDESC_OFFSET)
#define AVR32_UDDMA_ADDR(n)           (AVR32_UDDMA_BASE(n)+AVR32_UDDMA_ADDR_OFFSET)
#define AVR32_UDDMA_CONTROL(n)        (AVR32_UDDMA_BASE(n)+AVR32_UDDMA_CONTROL_OFFSET)
#define AVR32_UDDMA_STATUS(n)         (AVR32_UDDMA_BASE(n)+AVR32_UDDMA_STATUS_OFFSET)

#define AVR32_UDDMA1_NEXTDESC         (AVR32_USB_BASE+AVR32_UDDMA1_NEXTDESC_OFFSET)
#define AVR32_UDDMA1_ADDR             (AVR32_USB_BASE+AVR32_UDDMA1_ADDR_OFFSET)
#define AVR32_UDDMA1_CONTROL          (AVR32_USB_BASE+AVR32_UDDMA1_CONTROL_OFFSET)
#define AVR32_UDDMA1_STATUS           (AVR32_USB_BASE+AVR32_UDDMA1_STATUS_OFFSET)

#define AVR32_UDDMA2_NEXTDESC         (AVR32_USB_BASE+AVR32_UDDMA2_NEXTDESC_OFFSET)
#define AVR32_UDDMA2_ADDR             (AVR32_USB_BASE+AVR32_UDDMA2_ADDR_OFFSET)
#define AVR32_UDDMA2_CONTROL          (AVR32_USB_BASE+AVR32_UDDMA2_CONTROL_OFFSET)
#define AVR32_UDDMA2_STATUS           (AVR32_USB_BASE+AVR32_UDDMA2_STATUS_OFFSET)

#define AVR32_UDDMA3_NEXTDESC         (AVR32_USB_BASE+AVR32_UDDMA3_NEXTDESC_OFFSET)
#define AVR32_UDDMA3_ADDR             (AVR32_USB_BASE+AVR32_UDDMA3_ADDR_OFFSET)
#define AVR32_UDDMA3_CONTROL          (AVR32_USB_BASE+AVR32_UDDMA3_CONTROL_OFFSET)
#define AVR32_UDDMA3_STATUS           (AVR32_USB_BASE+AVR32_UDDMA3_STATUS_OFFSET)

#define AVR32_UDDMA4_NEXTDESC         (AVR32_USB_BASE+AVR32_UDDMA4_NEXTDESC_OFFSET)
#define AVR32_UDDMA4_ADDR             (AVR32_USB_BASE+AVR32_UDDMA4_ADDR_OFFSET )
#define AVR32_UDDMA4_CONTROL          (AVR32_USB_BASE+AVR32_UDDMA4_CONTROL_OFFSET)
#define AVR32_UDDMA4_STATUS           (AVR32_USB_BASE+AVR32_UDDMA4_STATUS_OFFSET)

#define AVR32_UDDMA5_NEXTDESC         (AVR32_USB_BASE+AVR32_UDDMA5_NEXTDESC_OFFSET)
#define AVR32_UDDMA5_ADDR             (AVR32_USB_BASE+AVR32_UDDMA5_ADDR_OFFSET)
#define AVR32_UDDMA5_CONTROL          (AVR32_USB_BASE+AVR32_UDDMA5_CONTROL_OFFSET)
#define AVR32_UDDMA5_STATUS           (AVR32_USB_BASE+AVR32_UDDMA5_STATUS_OFFSET)

#define AVR32_UDDMA6_NEXTDESC         (AVR32_USB_BASE+AVR32_UDDMA6_NEXTDESC_OFFSET)
#define AVR32_UDDMA6_ADDR             (AVR32_USB_BASE+AVR32_UDDMA6_ADDR_OFFSET)
#define AVR32_UDDMA6_CONTROL          (AVR32_USB_BASE+AVR32_UDDMA6_CONTROL_OFFSET)
#define AVR32_UDDMA6_STATUS           (AVR32_USB_BASE+AVR32_UDDMA6_STATUS_OFFSET)

#define AVR32_USBB_UHCON              (AVR32_USB_BASE+AVR32_USBB_UHCON_OFFSET)
#define AVR32_USBB_UHINT              (AVR32_USB_BASE+AVR32_USBB_UHINT_OFFSET)
#define AVR32_USBB_UHINTCLR           (AVR32_USB_BASE+AVR32_USBB_UHINTCLR_OFFSET)
#define AVR32_USBB_UHINTSET           (AVR32_USB_BASE+AVR32_USBB_UHINTSET_OFFSET)
#define AVR32_USBB_UHINTE             (AVR32_USB_BASE+AVR32_USBB_UHINTE_OFFSET)
#define AVR32_USBB_UHINTECLR          (AVR32_USB_BASE+AVR32_USBB_UHINTECLR_OFFSET)
#define AVR32_USBB_UHINTESET          (AVR32_USB_BASE+AVR32_USBB_UHINTESET_OFFSET)
#define AVR32_USBB_UPRST              (AVR32_USB_BASE+AVR32_USBB_UPRST_OFFSET)
#define AVR32_USBB_UHFNUM             (AVR32_USB_BASE+AVR32_USBB_UHFNUM_OFFSET)
#define AVR32_USBB_UHADDR1            (AVR32_USB_BASE+AVR32_USBB_UHADDR1_OFFSET)
#define AVR32_USBB_UHADDR2            (AVR32_USB_BASE+AVR32_USBB_UHADDR2_OFFSET)

#define AVR32_USBB_UPCFG(n)           (AVR32_USB_BASE+AVR32_USBB_UPCFG_OFFSET(n))
#define AVR32_USBB_UPCFG0             (AVR32_USB_BASE+AVR32_USBB_UPCFG0_OFFSET)
#define AVR32_USBB_UPCFG1             (AVR32_USB_BASE+AVR32_USBB_UPCFG1_OFFSET)
#define AVR32_USBB_UPCFG2             (AVR32_USB_BASE+AVR32_USBB_UPCFG2_OFFSET)
#define AVR32_USBB_UPCFG3             (AVR32_USB_BASE+AVR32_USBB_UPCFG3_OFFSET)
#define AVR32_USBB_UPCFG4             (AVR32_USB_BASE+AVR32_USBB_UPCFG4_OFFSET)
#define AVR32_USBB_UPCFG5             (AVR32_USB_BASE+AVR32_USBB_UPCFG5_OFFSET)
#define AVR32_USBB_UPCFG6             (AVR32_USB_BASE+AVR32_USBB_UPCFG6_OFFSET)

#define AVR32_USBB_UPSTA(n)           (AVR32_USB_BASE+AVR32_USBB_UPSTA_OFFSET(n))
#define AVR32_USBB_UPSTA0             (AVR32_USB_BASE+AVR32_USBB_UPSTA0_OFFSET)
#define AVR32_USBB_UPSTA1             (AVR32_USB_BASE+AVR32_USBB_UPSTA1_OFFSET)
#define AVR32_USBB_UPSTA2             (AVR32_USB_BASE+AVR32_USBB_UPSTA2_OFFSET)
#define AVR32_USBB_UPSTA3             (AVR32_USB_BASE+AVR32_USBB_UPSTA3_OFFSET)
#define AVR32_USBB_UPSTA4             (AVR32_USB_BASE+AVR32_USBB_UPSTA4_OFFSET)
#define AVR32_USBB_UPSTA5             (AVR32_USB_BASE+AVR32_USBB_UPSTA5_OFFSET)
#define AVR32_USBB_UPSTA6             (AVR32_USB_BASE+AVR32_USBB_UPSTA6_OFFSET)

#define AVR32_USBB_UPSTACLR(n)        (AVR32_USB_BASE+AVR32_USBB_UPSTACLR_OFFSET(n))
#define AVR32_USBB_UPSTA0CLR          (AVR32_USB_BASE+AVR32_USBB_UPSTA0CLR_OFFSET)
#define AVR32_USBB_UPSTA1CLR          (AVR32_USB_BASE+AVR32_USBB_UPSTA1CLR_OFFSET)
#define AVR32_USBB_UPSTA2CLR          (AVR32_USB_BASE+AVR32_USBB_UPSTA2CLR_OFFSET)
#define AVR32_USBB_UPSTA3CLR          (AVR32_USB_BASE+AVR32_USBB_UPSTA3CLR_OFFSET)
#define AVR32_USBB_UPSTA4CLR          (AVR32_USB_BASE+AVR32_USBB_UPSTA4CLR_OFFSET)
#define AVR32_USBB_UPSTA5CLR          (AVR32_USB_BASE+AVR32_USBB_UPSTA5CLR_OFFSET)
#define AVR32_USBB_UPSTA6CLR          (AVR32_USB_BASE+AVR32_USBB_UPSTA6CLR_OFFSET)

#define AVR32_USBB_UPSTASET(n)        (AVR32_USB_BASE+AVR32_USBB_UPSTASET_OFFSET(n))
#define AVR32_USBB_UPSTA0SET          (AVR32_USB_BASE+AVR32_USBB_UPSTA0SET_OFFSET)
#define AVR32_USBB_UPSTA1SET          (AVR32_USB_BASE+AVR32_USBB_UPSTA1SET_OFFSET)
#define AVR32_USBB_UPSTA2SET          (AVR32_USB_BASE+AVR32_USBB_UPSTA2SET_OFFSET)
#define AVR32_USBB_UPSTA3SET          (AVR32_USB_BASE+AVR32_USBB_UPSTA3SET_OFFSET)
#define AVR32_USBB_UPSTA4SET          (AVR32_USB_BASE+AVR32_USBB_UPSTA4SET_OFFSET)
#define AVR32_USBB_UPSTA5SET          (AVR32_USB_BASE+AVR32_USBB_UPSTA5SET_OFFSET)
#define AVR32_USBB_UPSTA6SET          (AVR32_USB_BASE+AVR32_USBB_UPSTA6SET_OFFSET)

#define AVR32_USBB_UPCON(n)           (AVR32_USB_BASE+AVR32_USBB_UPCON_OFFSET(n))
#define AVR32_USBB_UPCON0             (AVR32_USB_BASE+AVR32_USBB_UPCON0_OFFSET)
#define AVR32_USBB_UPCON1             (AVR32_USB_BASE+AVR32_USBB_UPCON1_OFFSET)
#define AVR32_USBB_UPCON2             (AVR32_USB_BASE+AVR32_USBB_UPCON2_OFFSET)
#define AVR32_USBB_UPCON3             (AVR32_USB_BASE+AVR32_USBB_UPCON3_OFFSET)
#define AVR32_USBB_UPCON4             (AVR32_USB_BASE+AVR32_USBB_UPCON4_OFFSET)
#define AVR32_USBB_UPCON5             (AVR32_USB_BASE+AVR32_USBB_UPCON5_OFFSET)
#define AVR32_USBB_UPCON6             (AVR32_USB_BASE+AVR32_USBB_UPCON6_OFFSET)

#define AVR32_USBB_UPCONSET(n)        (AVR32_USB_BASE+AVR32_USBB_UPCONSET_OFFSET(n))
#define AVR32_USBB_UPCON0SET          (AVR32_USB_BASE+AVR32_USBB_UPCON0SET_OFFSET)
#define AVR32_USBB_UPCON1SET          (AVR32_USB_BASE+AVR32_USBB_UPCON1SET_OFFSET)
#define AVR32_USBB_UPCON2SET          (AVR32_USB_BASE+AVR32_USBB_UPCON2SET_OFFSET)
#define AVR32_USBB_UPCON3SET          (AVR32_USB_BASE+AVR32_USBB_UPCON3SET_OFFSET)
#define AVR32_USBB_UPCON4SET          (AVR32_USB_BASE+AVR32_USBB_UPCON4SET_OFFSET)
#define AVR32_USBB_UPCON5SET          (AVR32_USB_BASE+AVR32_USBB_UPCON5SET_OFFSET)
#define AVR32_USBB_UPCON6SET          (AVR32_USB_BASE+AVR32_USBB_UPCON6SET_OFFSET)

#define AVR32_USBB_UPCONCLR(n)        (AVR32_USB_BASE+AVR32_USBB_UPCONCLR_OFFSET(n))
#define AVR32_USBB_UPCON0CLR          (AVR32_USB_BASE+AVR32_USBB_UPCON0CLR_OFFSET)
#define AVR32_USBB_UPCON1CLR          (AVR32_USB_BASE+AVR32_USBB_UPCON1CLR_OFFSET)
#define AVR32_USBB_UPCON2CLR          (AVR32_USB_BASE+AVR32_USBB_UPCON2CLR_OFFSET)
#define AVR32_USBB_UPCON3CLR          (AVR32_USB_BASE+AVR32_USBB_UPCON3CLR_OFFSET)
#define AVR32_USBB_UPCON4CLR          (AVR32_USB_BASE+AVR32_USBB_UPCON4CLR_OFFSET)
#define AVR32_USBB_UPCON5CLR          (AVR32_USB_BASE+AVR32_USBB_UPCON5CLR_OFFSET)
#define AVR32_USBB_UPCON6CLR          (AVR32_USB_BASE+AVR32_USBB_UPCON6CLR_OFFSET)

#define AVR32_USBB_UPINRQ(n)          (AVR32_USB_BASE+AVR32_USBB_UPINRQ_OFFSET(n))
#define AVR32_USBB_UPINRQ0            (AVR32_USB_BASE+AVR32_USBB_UPINRQ0_OFFSET)
#define AVR32_USBB_UPINRQ1            (AVR32_USB_BASE+AVR32_USBB_UPINRQ1_OFFSET)
#define AVR32_USBB_UPINRQ2            (AVR32_USB_BASE+AVR32_USBB_UPINRQ2_OFFSET)
#define AVR32_USBB_UPINRQ3            (AVR32_USB_BASE+AVR32_USBB_UPINRQ3_OFFSET)
#define AVR32_USBB_UPINRQ4            (AVR32_USB_BASE+AVR32_USBB_UPINRQ4_OFFSET)
#define AVR32_USBB_UPINRQ5            (AVR32_USB_BASE+AVR32_USBB_UPINRQ5_OFFSET)
#define AVR32_USBB_UPINRQ6            (AVR32_USB_BASE+AVR32_USBB_UPINRQ6_OFFSET)

#define AVR32_USBB_UPERR(n)           (AVR32_USB_BASE+AVR32_USBB_UPERR_OFFSET(n))
#define AVR32_USBB_UPERR0             (AVR32_USB_BASE+AVR32_USBB_UPERR0_OFFSET)
#define AVR32_USBB_UPERR1             (AVR32_USB_BASE+AVR32_USBB_UPERR1_OFFSET)
#define AVR32_USBB_UPERR2             (AVR32_USB_BASE+AVR32_USBB_UPERR2_OFFSET)
#define AVR32_USBB_UPERR3             (AVR32_USB_BASE+AVR32_USBB_UPERR3_OFFSET)
#define AVR32_USBB_UPERR4             (AVR32_USB_BASE+AVR32_USBB_UPERR4_OFFSET)
#define AVR32_USBB_UPERR5             (AVR32_USB_BASE+AVR32_USBB_UPERR5_OFFSET)
#define AVR32_USBB_UPERR6             (AVR32_USB_BASE+AVR32_USBB_UPERR6_OFFSET)

#define AVR32_UHDMA_BASE(n)           (AVR32_USB_BASE+AVR32_UHDMA_OFFSET(n))
#define AVR32_UHDMA_NEXTDESC(n)       (AVR32_UHDMA_BASE(n)+AVR32_UHDMA_NEXTDESC_OFFSET)
#define AVR32_UHDMA_ADDR(n)           (AVR32_UHDMA_BASE(n)+AVR32_UHDMA_ADDR_OFFSET)
#define AVR32_UHDMA_CONTROL(n)        (AVR32_UHDMA_BASE(n)+AVR32_UHDMA_CONTROL_OFFSET)
#define AVR32_UHDMA_STATUS(n)         (AVR32_UHDMA_BASE(n)+AVR32_UHDMA_STATUS_OFFSET)

#define AVR32_UHDMA1_NEXTDESC         (AVR32_USB_BASE+AVR32_UHDMA1_NEXTDESC_OFFSET)
#define AVR32_UHDMA1_ADDR             (AVR32_USB_BASE+AVR32_UHDMA1_ADDR_OFFSET)
#define AVR32_UHDMA1_CONTROL          (AVR32_USB_BASE+AVR32_UHDMA1_CONTROL_OFFSET)
#define AVR32_UHDMA1_STATUS           (AVR32_USB_BASE+AVR32_UHDMA1_STATUS_OFFSET)

#define AVR32_UHDMA2_NEXTDESC         (AVR32_USB_BASE+AVR32_UHDMA2_NEXTDESC_OFFSET)
#define AVR32_UHDMA2_ADDR             (AVR32_USB_BASE+AVR32_UHDMA2_ADDR_OFFSET)
#define AVR32_UHDMA2_CONTROL          (AVR32_USB_BASE+AVR32_UHDMA2_CONTROL_OFFSET)
#define AVR32_UHDMA2_STATUS           (AVR32_USB_BASE+AVR32_UHDMA2_STATUS_OFFSET)

#define AVR32_UHDMA3_NEXTDESC         (AVR32_USB_BASE+AVR32_UHDMA3_NEXTDESC_OFFSET)
#define AVR32_UHDMA3_ADDR             (AVR32_USB_BASE+AVR32_UHDMA3_ADDR_OFFSET)
#define AVR32_UHDMA3_CONTROL          (AVR32_USB_BASE+AVR32_UHDMA3_CONTROL_OFFSET)
#define AVR32_UHDMA3_STATUS           (AVR32_USB_BASE+AVR32_UHDMA3_STATUS_OFFSET)

#define AVR32_UHDMA4_NEXTDESC         (AVR32_USB_BASE+AVR32_UHDMA4_NEXTDESC_OFFSET)
#define AVR32_UHDMA4_ADDR             (AVR32_USB_BASE+AVR32_UHDMA4_ADDR_OFFSET)
#define AVR32_UHDMA4_CONTROL          (AVR32_USB_BASE+AVR32_UHDMA4_CONTROL_OFFSET)
#define AVR32_UHDMA4_STATUS           (AVR32_USB_BASE+AVR32_UHDMA4_STATUS_OFFSET)

#define AVR32_UHDMA5_NEXTDESC         (AVR32_USB_BASE+AVR32_UHDMA5_NEXTDESC_OFFSET)
#define AVR32_UHDMA5_ADDR             (AVR32_USB_BASE+AVR32_UHDMA5_ADDR_OFFSET)
#define AVR32_UHDMA5_CONTROL          (AVR32_USB_BASE+AVR32_UHDMA5_CONTROL_OFFSET)
#define AVR32_UHDMA5_STATUS           (AVR32_USB_BASE+AVR32_UHDMA5_STATUS_OFFSET)

#define AVR32_UHDMA6_NEXTDESC         (AVR32_USB_BASE+AVR32_UHDMA6_NEXTDESC_OFFSET)
#define AVR32_UHDMA6_ADDR             (AVR32_USB_BASE+AVR32_UHDMA6_ADDR_OFFSET)
#define AVR32_UHDMA6_CONTROL          (AVR32_USB_BASE+AVR32_UHDMA6_CONTROL_OFFSET)
#define AVR32_UHDMA6_STATUS           (AVR32_USB_BASE+AVR32_UHDMA6_STATUS_OFFSET)

#define AVR32_USBB_USBCON             (AVR32_USB_BASE+AVR32_USBB_USBCON_OFFSET)
#define AVR32_USBB_USBSTA             (AVR32_USB_BASE+AVR32_USBB_USBSTA_OFFSET)
#define AVR32_USBB_USBSTACLR          (AVR32_USB_BASE+AVR32_USBB_USBSTACLR_OFFSET)
#define AVR32_USBB_USBSTASET          (AVR32_USB_BASE+AVR32_USBB_USBSTASET_OFFSET)
#define AVR32_USBB_UVERS              (AVR32_USB_BASE+AVR32_USBB_UVERS_OFFSET)
#define AVR32_USBB_UFEATURES          (AVR32_USB_BASE+AVR32_USBB_UFEATURES_OFFSET)
#define AVR32_USBB_UADDRSIZE          (AVR32_USB_BASE+AVR32_USBB_UADDRSIZE_OFFSET)
#define AVR32_USBB_UNAME1             (AVR32_USB_BASE+AVR32_USBB_UNAME1_OFFSET)
#define AVR32_USBB_UNAME2             (AVR32_USB_BASE+AVR32_USBB_UNAME2_OFFSET)
#define AVR32_USBB_USBFSM             (AVR32_USB_BASE+AVR32_USBB_USBFSM_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Device General Control Register */
#define USBB_UDCON_
/* Device Global Interrupt Register */
#define USBB_UDINT_
/* Device Global Interrupt Clear Register */
#define USBB_UDINTCLR_
/* Device Global Interrupt Set Register */
#define USBB_UDINTSET_
/* Device Global Interrupt Enable Register */
#define USBB_UDINTE_
/* Device Global Interrupt Enable Clear Register */
#define USBB_UDINTECLR_
/* Device Global Interrupt Enable Set Register */
#define USBB_UDINTESET_
/* Endpoint Enable/Reset Register */
#define USBB_UERST_
/* Device Frame Number Register */
#define USBB_UDFNUM_

/* Endpoint Configuration Register */
#define USBB_UECFG_
/* Endpoint Status Register */
#define USBB_UESTA_
/* Endpoint Status Clear Register */
#define USBB_UESTACLR_
/* Endpoint  Status Set Register */
#define USBB_UESTASET_
/* Endpoint Control Register */
#define USBB_UECON_
/* Endpoint Control Set Register */
#define USBB_UECONSET_
/* Endpoint Control Clear Register */
#define USBB_UECONCLR_

/* Device DMA Channel Next Descriptor Address Register */
#define UDDMA_NEXTDESC_
/* Device DMA Channel HSB Address Register */
#define UDDMA_ADDR_
/* Device DMA Channel Control Register */
#define UDDMA_CONTROL_
/* Device DMA Channel Status Register */
#define UDDMA_STATUS_

/* Host General Control Register */
#define USBB_UHCON_
/* Host Global Interrupt Register */
#define USBB_UHINT_
/* Host Global Interrupt Clear Register */
#define USBB_UHINTCLR_
/* Host Global Interrupt Set Register */
#define USBB_UHINTSET_
/* Host Global Interrupt Enable Register */
#define USBB_UHINTE_
/* Host Global Interrupt Enable Clear Register */
#define USBB_UHINTECLR_
/* Host Global Interrupt Enable Set Register */
#define USBB_UHINTESET_
/* Pipe Enable/Reset Register */
#define USBB_UPRST_
/* Host Frame Number Register */
#define USBB_UHFNUM_
/* Host Address 1 Register */
#define USBB_UHADDR1_
/* Host Address 2 Register */
#define USBB_UHADDR2_

/* Pipe Configuration Register */
#define USBB_UPCFG_
/* Pipe Status Register */
#define USBB_UPSTA_
/* Pipe Status Clear Register */
#define USBB_UPSTACLR_
/* Pipe Status Set Register */
#define USBB_UPSTASET_
/* Pipe Control Register */
#define USBB_UPCON_
/* Pipe Control Set Register */
#define USBB_UPCONSET_
/* Pipe Control Clear Register */
#define USBB_UPCONCLR_
/* Pipe IN Request Register */
#define USBB_UPINRQ_
/* Pipe Error Register */
#define USBB_UPERR_

/* Host DMA Channel Next Descriptor Address Register */
#define UHDMA_NEXTDESC_
/* Host DMA Channel HSB Address Register */
#define UHDMA_ADDR_
/* Host DMA Channel Control Register */
#define UHDMA_CONTROL_
/* Host DMA Channel Status Register */
#define UHDMA_STATUS_

/* General Control Register */
#define USBB_USBCON_

/* General Status Register */
/* General Status Clear Register */
/* General Status Set Register */

#define USBB_USBSTA_IDTI              (1 << 0)  /* Bit 0:  ID Transition Interrupt */
#define USBB_USBSTA_VBUSTI            (1 << 1)  /* Bit 1:  VBus Transition Interrupt */
#define USBB_USBSTA_VBERRI            (1 << 3)  /* Bit 3:  VBus Error Interrupt */
#define USBB_USBSTA_BCERRI            (1 << 4)  /* Bit 4:  B-Connection Error Interrupt */
#define USBB_USBSTA_ROLEEXI           (1 << 5)  /* Bit 5:  Role Exchange Interrupt */
#define USBB_USBSTA_STOI              (1 << 7)  /* Bit 7:  Suspend Time-Out Interrupt */
#define USBB_USBSTA_VBUSRQ            (1 << 9)  /* Bit 8:  VBus Request */
#define USBB_USBSTA_ID                (1 << 10) /* Bit 10: USB_ID Pin State (read-only) */
#define USBB_USBSTA_VBUS              (1 << 11) /* Bit 11: VBus Level (read-only) */
#define USBB_USBSTA_SPEED_SHIFT       (12)      /* Bits 12-13:  Speed Status (read-only) */
#define USBB_USBSTA_SPEED_MASK        (3 << USBB_USBSTA_SPEED_SHIFT)
#  define USBB_USBSTA_SPEED_FULL      (0 << USBB_USBSTA_SPEED_SHIFT) /* Full-Speed mode */
#  define USBB_USBSTA_SPEED_FULL      (2 << USBB_USBSTA_SPEED_SHIFT) /* Low-Speed mode */

/* IP Version Register */

#define USBB_UVERS_SHIFT              (0)       /* Bits 0-11: Version Number */
#define USBB_UVERS_MASK               (0xfff << USBB_UVERS_SHIFT)
#define USBB_UVERS_VARIANT_SHIFT      (16)      /* Bits 16-19: Variant Number */
#define USBB_UVERS_VARIANT_MASK       (15 << USBB_UVERS_VARIANT_SHIFT)

/* IP Features Register */

#define USBB_UFEAT_EPTNBRMAX_SHIFT    (0)       /* Bits 0-3: Maximal Number of Pipes/Endpoints */
#define USBB_UFEAT_EPTNBRMAX_MASK     (15 << USBB_UFEAT_EPTNBRMAX_SHIFT)
#  define USBB_UFEAT_EPTNBRMAX_16     (0 << USBB_UFEAT_EPTNBRMAX_SHIFT) /* 16 is a special case */
#define USBB_UFEAT_DMACHANNBR_SHIFT   (4)       /* Bits 4-6: Number of DMA Channels */
#define USBB_UFEAT_DMACHANNBR_MASK    (7 << USBB_UFEAT_DMACHANNBR_SHIFT)
#define USBB_UFEAT_DMABUFFERSZ        (1 << 7)  /* Bit 7:  DMA Buffer Size */
#define USBB_UFEAT_DMAWDDEPTH_SHIFT   (8)       /* Bits 8-11: DMA FIFO Depth in Words */
#define USBB_UFEAT_DMAWDDEPTH_MASK    (15 << USBB_UFEAT_DMAWDDEPTH_SHIFT)
#  define USBB_UFEAT_DMAWDDEPTH_16    (0 << USBB_UFEAT_DMAWDDEPTH_SHIFT) /* 16 is a special case */
#define USBB_UFEAT_FIFOMAXSZ_SHIFT    (12)      /* Bits 12-14: Maximal FIFO Size */
#define USBB_UFEAT_FIFOMAXSZ_MASK     (7 << USBB_UFEAT_FIFOMAXSZ_SHIFT)
#  define USBB_UFEAT_FIFOMAXSZ_LT256  (0 << USBB_UFEAT_FIFOMAXSZ_SHIFT) /* < 256 bytes */
#  define USBB_UFEAT_FIFOMAXSZ_LT512  (1 << USBB_UFEAT_FIFOMAXSZ_SHIFT) /* < 512 bytes */
#  define USBB_UFEAT_FIFOMAXSZ_LT1K   (2 << USBB_UFEAT_FIFOMAXSZ_SHIFT) /* < 1024 bytes */
#  define USBB_UFEAT_FIFOMAXSZ_LT2K   (3 << USBB_UFEAT_FIFOMAXSZ_SHIFT) /* < 2048 bytes */
#  define USBB_UFEAT_FIFOMAXSZ_LT4K   (4 << USBB_UFEAT_FIFOMAXSZ_SHIFT) /* < 4096 bytes */
#  define USBB_UFEAT_FIFOMAXSZ_LT8K   (5 << USBB_UFEAT_FIFOMAXSZ_SHIFT) /* < 8192 bytes */
#  define USBB_UFEAT_FIFOMAXSZ_LT16K  (6 << USBB_UFEAT_FIFOMAXSZ_SHIFT) /* < 16384 bytes */
#  define USBB_UFEAT_FIFOMAXSZ_GE16K  (7 << USBB_UFEAT_FIFOMAXSZ_SHIFT) /* >= 16384 bytes */
#define USBB_UFEAT_BWRDPRAM           (1 << 15) /* Bit 15: DPRAM Byte-Write Capability */

/* IP PB Address Size Register */
#define USBB_UADDRSIZE_
/* IP Name Register 1 */
#define USBB_UNAME1_
/* IP Name Register 2 */
#define USBB_UNAME2_
/* USB Finite State Machine Status Register */
#define USBB_USBFSM_

              (1 << xxx)  /* Bit xxx:  
_SHIFT       (xxx)       /* Bits xxx-xxx: 
_MASK        (xxx << xxx)

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


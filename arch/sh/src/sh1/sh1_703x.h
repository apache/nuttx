/************************************************************************************
 * arch/sh/src/sh1/sh1_703x.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_SH_SRC_SH1_703X_H
#define __ARCH_SH_SRC_SH1_703X_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Memory-mapped register addresses *************************************************/

/* Serial Communications interface (SCI) */

#define SH1_SCI0_BASE       (0x05fffec0)
#define SH1_SCI1_BASE       (0x05fffec8)

#define SH1_SCI_SMR_OFFSET  (0)  /* Serial Mode Register (8-bits wide) */
#define SH1_SCI_BRR_OFFSET  (1)  /* Bit Rate Registr (8-bits wide) */
#define SH1_SCI_SCR_OFFSET  (2)  /* Serial Control Register (8-bits wide) */
#define SH1_SCI_TDR_OFFSET  (3)  /* Transmit Data Register (8-bits wide) */
#define SH1_SCI_SSR_OFFSET  (4)  /* Serial Status Register (8-bits wide) */
#define SH1_SCI_RDR_OFFSET  (5)  /* Receive Data Register (8-bits wide) */

#define SH1_SCI0_SMR        (SH1_SCI0_BASE+SH1_SCI_SMR_OFFSET)
#define SH1_SCI0_BRR        (SH1_SCI0_BASE+SH1_SCI_BRR_OFFSET)
#define SH1_SCI0_SCR        (SH1_SCI0_BASE+SH1_SCI_SCR_OFFSET)
#define SH1_SCI0_TDR        (SH1_SCI0_BASE+SH1_SCI_TDR_OFFSET)
#define SH1_SCI0_SSR        (SH1_SCI0_BASE+SH1_SCI_SSR_OFFSET)
#define SH1_SCI0_RDR        (SH1_SCI0_BASE+SH1_SCI_RDR_OFFSET)

#define SH1_SCI1_SMR        (SH1_SCI1_BASE+SH1_SCI_SMR_OFFSET)
#define SH1_SCI1_BRR        (SH1_SCI1_BASE+SH1_SCI_BRR_OFFSET)
#define SH1_SCI1_SCR        (SH1_SCI1_BASE+SH1_SCI_SCR_OFFSET)
#define SH1_SCI1_TDR        (SH1_SCI1_BASE+SH1_SCI_TDR_OFFSET)
#define SH1_SCI1_SSR        (SH1_SCI1_BASE+SH1_SCI_SSR_OFFSET)
#define SH1_SCI1_RDR        (SH1_SCI1_BASE+SH1_SCI_RDR_OFFSET)

/* A/D */

#define SH1_AD_ADDRA        (0x05fffee0)  /* 16-bits wide */
#define SH1_AD_DRAH         (0x05fffee0)  /* 8-bits wide */
#define SH1_AD_DRAL         (0x05fffee1)  /* 8-bits wide */
#define SH1_AD_DRB          (0x05fffee2)  /* 16-bits wide */
#define SH1_AD_DRBH         (0x05fffee2)  /* 8-bits wide */
#define SH1_AD_DRBL         (0x05fffee3)  /* 8-bits wide */
#define SH1_AD_DRC          (0x05fffee4)  /* 16-bits wide */
#define SH1_AD_DRCH         (0x05fffee4)  /* 8-bits wide */
#define SH1_AD_DRCL         (0x05fffee5)  /* 8-bits wide */
#define SH1_AD_DRD          (0x05fffee6)  /* 16-bits wide */
#define SH1_AD_DRDH         (0x05fffee6)  /* 8-bits wide */
#define SH1_AD_DRDL         (0x05fffee7)  /* 8-bits wide */
#define SH1_AD_CSR          (0x05fffee8)  /* 8-bits wide */
#define SH1_AD_CR           (0x05fffee9)  /* 8-bits wide */

/* Integrated Timer/Pulse Unit (ITU) */

/* ITU shared */

#define SH1_ITU_TSTR        (0x05ffff00)  /* 8-bits wide */
#define SH1_ITU_TSNC        (0x05ffff01)  /* 16-bits wide */
#define SH1_ITU_TMDR        (0x05ffff02)  /* 16-bits wide */
#define SH1_ITU_TFCR        (0x05ffff03)  /* 16-bits wide */

/* ITU channel 0 */

#define SH1_ITU0_TCR        (0x05ffff04)  /* 8-bits wide */
#define SH1_ITU0_TIOR       (0x05ffff05)  /* 8-bits wide */
#define SH1_ITU0_TIER       (0x05ffff06)  /* 8-bits wide */
#define SH1_ITU0_TSR        (0x05ffff07)  /* 8-bits wide */
#define SH1_ITU0_TCNT       (0x05ffff08)  /* 16-bits wide */
#define SH1_ITU0_GRA        (0x05ffff0a)  /* 16-bits wide */
#define SH1_ITU0_GRB        (0x05ffff0c)  /* 16-bits wide */

/* ITU channel 1 */

#define SH1_ITU1_TCR        (0x05ffff0e)  /* 8-bits wide */
#define SH1_ITU1_TIOR       (0x05ffff0f)  /* 8-bits wide */
#define SH1_ITU1_TIER       (0x05ffff10)  /* 8-bits wide */
#define SH1_ITU1_TSR        (0x05ffff11)  /* 8-bits wide */
#define SH1_ITU1_TCNT       (0x05ffff12)  /* 16-bits wide */
#define SH1_ITU1_GRA        (0x05ffff14)  /* 16-bits wide */
#define SH1_ITU1_GRB        (0x05ffff16)  /* 16-bits wide */

/* ITU channel 2 */

#define SH1_ITU2_TCR        (0x05ffff18)  /* 8-bits wide */
#define SH1_ITU2_TIOR       (0x05ffff19)  /* 8-bits wide */
#define SH1_ITU2_TIER       (0x05ffff1a)  /* 8-bits wide */
#define SH1_ITU2_TSR        (0x05ffff1b)  /* 8-bits wide */
#define SH1_ITU2_TCNT       (0x05ffff1c)  /* 16-bits wide */
#define SH1_ITU2_GRA        (0x05ffff1e)  /* 16-bits wide */
#define SH1_ITU2_GRB        (0x05ffff20)  /* 16-bits wide */

/* ITU channel 3 */

#define SH1_ITU3_TCR        (0x05ffff22)  /* 8-bits wide */
#define SH1_ITU3_TIOR       (0x05ffff23)  /* 8-bits wide */
#define SH1_ITU3_TIER       (0x05ffff24)  /* 8-bits wide */
#define SH1_ITU3_TSR        (0x05ffff25)  /* 8-bits wide */
#define SH1_ITU3_TCNT       (0x05ffff26)  /* 16-bits wide */
#define SH1_ITU3_GRA        (0x05ffff28)  /* 16-bits wide */
#define SH1_ITU3_GRB        (0x05ffff2a)  /* 16-bits wide */
#define SH1_ITU3_BRA        (0x05ffff2c)  /* 16-bits wide */
#define SH1_ITU3_BRB3       (0x05ffff2e)  /* 16-bits wide */

/* ITU channels 0-4 shared */

#define SH1_ITU_TOCR        (0x05ffff31)  /* 8-bits wide */

/* ITU channel 4 */

#define SH1_ITU4_TCR        (0x05ffff32)  /* 8-bits wide */
#define SH1_ITU4_TIOR       (0x05ffff33)  /* 8-bits wide */
#define SH1_ITU4_TIER       (0x05ffff34)  /* 8-bits wide */
#define SH1_ITU4_TSR        (0x05ffff35)  /* 8-bits wide */
#define SH1_ITU4_TCNT       (0x05ffff36)  /* 16-bits wide */
#define SH1_ITU4_GRA        (0x05ffff38)  /* 16-bits wide */
#define SH1_ITU4_GRB        (0x05ffff3a)  /* 16-bits wide */
#define SH1_ITU4_BRA        (0x05ffff3c)  /* 16-bits wide */
#define SH1_ITU4_BRB        (0x05ffff3e)  /* 16-bits wide */

/* DMA controller (DMAC) */

/* DMAC channels 0-3 shared */

#define SH1_DMAOR           (0x05ffff48)  /* 16-bits wide */

/* DMAC channel 0 */

#define SH1_DMA0_SAR0       (0x05ffff40)  /* 32-bits wide */
#define SH1_DMA0_DAR0       (0x05ffff44)  /* 32-bits wide */
#define SH1_DMA0_TCR0       (0x05ffff4a)  /* 16-bits wide */
#define SH1_DMA0_CHCR0      (0x05ffff4e)  /* 16-bits wide */

/* DMAC channel 1 */

#define SH1_DMA1_SAR        (0x05ffff50)  /* 32-bits wide */
#define SH1_DMA1_DAR        (0x05ffff54)  /* 32-bits wide */
#define SH1_DMA1_TCR        (0x05fffF5a)  /* 16-bits wide */
#define SH1_DMA1_CHCR       (0x05ffff5e)  /* 16-bits wide */

/* DMAC channel 2 */

#define SH1_DMA2_SAR        (0x05ffff60)  /* 32-bits wide */
#define SH1_DMA2_DAR        (0x05ffff64)  /* 32-bits wide */
#define SH1_DMA2_TCR        (0x05fffF6a)  /* 16-bits wide */
#define SH1_DMA2_CHCR       (0x05ffff6e)  /* 16-bits wide */

/* DMAC channel 3 */

#define SH1_DMA3_SAR        (0x05ffff70)  /* 32-bits wide */
#define SH1_DMA3_DAR        (0x05ffff74)  /* 32-bits wide */
#define SH1_DMA3_TCR        (0x05fffF7a)  /* 16-bits wide */
#define SH1_DMA3_CHCR       (0x05ffff7e)  /* 16-bits wide */

/* Interrupt Controller (INTC) */

#define SH1_INTC_IPRA       (0x05ffff84)  /* Interrupt priority register A (16-bits wide) */
#define SH1_INTC_IPRB       (0x05ffff86)  /* Interrupt priority register B (16-bits wide) */
#define SH1_INTC_IPRC       (0x05ffff88)  /* Interrupt priority register C (16-bits wide) */
#define SH1_INTC_IPRD       (0x05ffff8a)  /* Interrupt priority register D (16-bits wide) */
#define SH1_INTC_IPRE       (0x05ffff8c)  /* Interrupt priority register E (16-bits wide) */
#define SH1_INTC_ICR        (0x05ffff8e)  /* Interrupt control register (16-bits wide) */

/* User Break Controller (UBC) */

#define SH1_UBC_BARH        (0x05ffff90)  /* 16-bits wide */
#define SH1_UBC_BARL        (0x05ffff92)  /* 16-bits wide */
#define SH1_UBC_BAMRH       (0x05ffff94)  /* 16-bits wide */
#define SH1_UBC_BAMRL       (0x05ffff96)  /* 16-bits wide */
#define SH1_UBC_BBR         (0x05ffff98)  /* 16-bits wide */

/*Bus State Controller (BSC) */

#define SH1_BSC_BCR         (0x05ffffa0)  /* 16-bits wide */
#define SH1_BSC_WCR1        (0x05ffffa2)  /* 16-bits wide */
#define SH1_BSC_WCR2        (0x05ffffa4)  /* 16-bits wide */
#define SH1_BSC_WCR3        (0x05ffffa6)  /* 16-bits wide */
#define SH1_BSC_DCR         (0x05ffffa8)  /* 16-bits wide */
#define SH1_BSC_PCR         (0x05ffffaa)  /* 16-bits wide */
#define SH1_BSC_RCR         (0x05ffffac)  /* 16-bits wide */
#define SH1_BSC_RTCSR       (0x05ffffae)  /* 16-bits wide */
#define SH1_BSC_RTCNT       (0x05ffffb0)  /* 16-bits wide */
#define SH1_BSC_RTCOR       (0x05ffffb2)  /* 16-bits wide */

/* Watchdog Timer (WDT) */

#define SH1_WDT_TCSR        (0x05ffffb8)  /* 8-bits wide */
#define SH1_WDT_TCNT        (0x05ffffb9)  /* 8-bits wide */
#define SH1_WDT_RSTCSR      (0x05ffffbb)  /* 8-bits wide */

/* Power down state */

#define SH1_PDT_SBYCR       (0x05ffffbc)  /* 8-bits wide */

/* Port A */

#define SH1_PORTA_DR        (0x05ffffc0)  /* 16-bits wide */

/* Port B */

#define SH1_PORTB_DR        (0x05ffffc2)  /* 16-bits wide */

/* Pin Function Controller (PFC) */

#define SH1_PFC_PAIOR       (0x05ffffc4)  /* 16-bits wide */
#define SH1_PFC_PBIOR       (0x05ffffc6)  /* 16-bits wide */
#define SH1_PFC_PACR1       (0x05ffffc8)  /* 16-bits wide */
#define SH1_PFC_PACR2       (0x05ffffca)  /* 16-bits wide */
#define SH1_PFC_PBCR1       (0x05ffffcc)  /* 16-bits wide */
#define SH1_PFC_PBCR2       (0x05ffffce)  /* 16-bits wide */

/* Port C */

#define SH1_PORTC_DR        (0x05ffffd0)  /* 16-bits wide */

/* Pin Function Controller (PFC, cont'd) */

#define SH1_PFC_CASCR       (0x05ffffee)  /* 16-bits wide */

/* Timing Pattern Controller (TPC) */

#define SH1_TPC_TPMR        (0x05fffff0)  /* 16-bits wide */
#define SH1_TPC_TPCR        (0x05fffff1)  /* 16-bits wide */
#define SH1_TPC_NDERH       (0x05fffff2)  /* 16-bits wide */
#define SH1_TPC_NDERL       (0x05fffff3)  /* 16-bits wide */
#define SH1_TPC_NDRB0       (0x05fffff4)  /* 8-bits wide */
#define SH1_TPC_NDRA0       (0x05fffff5)  /* 8-bits wide */
#define SH1_TPC_NDRB1       (0x05fffff6)  /* 8-bits wide */
#define SH1_TPC_NDRA1       (0x05fffff7)  /* 8-bits wide */

/* Register bit definitions *********************************************************/

/* Serial Communications interface (SCI) */

#define SH1_SCISMR_CKSMASK  (0x03)        /* Bit 0-1: Internal clock source */
#define SH1_SCISMR_DIV1     (0x00)        /*   System clock (phi) */
#define SH1_SCISMR_DIV4     (0x01)        /*   phi/4 */
#define SH1_SCISMR_DIV16    (0x02)        /*   phi/16 */
#define SH1_SCISMR_DIV64    (0x03)        /*   phi/64 */
#define SH1_SCISMR_MP       (0x04)        /* Bit 2: Multiprocessor select */
#define SH1_SCISMR_STOP     (0x08)        /* Bit 3: 0:One stop bit, 1:Two stop bits */
#define SH1_SCISMR_OE       (0x10)        /* Bit 4: 0:Even parity, 1:Odd parity */
#define SH1_SCISMR_PE       (0x20)        /* Bit 5: Parity enable */
#define SH1_SCISMR_CHR      (0x40)        /* Bit 6: 0:8-bit data, 1:7-bit data */
#define SH1_SCISMR_CA       (Ox80)        /* Bit 7: 0:Asynchronous, 1:clocked synchronous */

#define SH1_SCISCR_CKEMASK  (0x03)        /* Bit 0-1: Internal clock source */
                                          /* Asynchronous mode: */
#define SH1_SCISCR_AISIN    (0x00)        /*   Internal clock, SCK pin used for input pin */
#define SH1_SCISCR_AISOUT   (0x01)        /*   Internal clock, SCK pin used for clock output */
#define SH1_SCISCR_AXSIN1   (0x02)        /*   External clock, SCK pin used for clock input */
#define SH1_SCISCR_AXSIN2   (0x03)        /*   External clock, SCK pin used for clock input */
                                          /* Synchronous mode: */
#define SH1_SCISCR_SISOUT1  (0x00)        /*   Internal clock, SCK pin used for input pin */
#define SH1_SCISCR_SISOUT2  (0x01)        /*   Internal clock, SCK pin used for clock output */
#define SH1_SCISCR_SXSIN1   (0x02)        /*   External clock, SCK pin used for clock input */
#define SH1_SCISCR_SXSIN2   (0x03)        /*   External clock, SCK pin used for clock input */
#define SH1_SCISCR_TEIE     (0x04)        /* Bit 2: 1=Transmit end interrupt enable */
#define SH1_SCISCR_MPIE     (0x08)        /* Bit 3: 1=Multiprocessor interrupt enable */
#define SH1_SCISCR_RE       (0x10)        /* Bit 4: 1=Receiver enable */
#define SH1_SCISCR_TE       (0x20)        /* Bit 5: 1=Transmitter enable */
#define SH1_SCISCR_RIE      (0x40)        /* Bit 6: 1=Recieve-data-full interrupt enable */
#define SH1_SCISCR_TIE      (Ox80)        /* Bit 7: 1=Transmit-data-empty interrupt enable */
#define SH1_SCISCR_ALLINTS  (0xcc)

#define SH1_SCISSR_MPBT     (0x01)        /* Bit 0: Multi-processor Bit in Transmit data */
#define SH1_SCISSR_MPB      (0x02)        /* Bit 1: Multi-processor Bit in receive data */
#define SH1_SCISSR_TEND     (0x04)        /* Bit 2: End of transmission */
#define SH1_SCISSR_PER      (0x08)        /* Bit 3: Receive parity error */
#define SH1_SCISSR_FER      (0x10)        /* Bit 4: Receive framing error */
#define SH1_SCISSR_ORER     (0x20)        /* Bit 5: Receive overrun error */
#define SH1_SCISSR_RDRF     (0x40)        /* Bit 6: RDR contains valid received data */
#define SH1_SCISSR_TDRE     (0x80)        /* Bit 7: TDR does not contain valid transmit data */

/* Interrupt Controller (INTC) */

#define SH1_IPRA_IRQ3MASK   (0x000f)      /* Bits 0-3: IRQ3 */
#define SH1_IPRA_IRQ3SHIFT  (0)
#define SH1_IPRA_IRQ2MASK   (0x00f0)      /* Bits 4-7: IRQ2 */
#define SH1_IPRA_IRQ2SHIFT  (4)
#define SH1_IPRA_IRQ1MASK   (0x0f00)      /* Bits 8-11: IRQ1 */
#define SH1_IPRA_IRQ1SHIFT  (8)
#define SH1_IPRA_IRQ0MASK   (0xf000)      /* Bits 12-15: IRQ0 */
#define SH1_IPRA_IRQ0SHIFT  (12)

#define SH1_IPRB_IRQ7MASK   (0x000f)      /* Bits 0-3: IRQ7 */
#define SH1_IPRB_IRQ7SHIFT  (0)
#define SH1_IPRB_IRQ6MASK   (0x00f0)      /* Bits 4-7: IRQ6 */
#define SH1_IPRB_IRQ6SHIFT  (4)
#define SH1_IPRB_IRQ5MASK   (0x0f00)      /* Bits 8-11: IRQ5 */
#define SH1_IPRB_IRQ5SHIFT  (8)
#define SH1_IPRB_IRQ4MASK   (0xf000)      /* Bits 12-15: IRQ4 */
#define SH1_IPRB_IRQ4SHIFT  (12)

#define SH1_IPRC_ITU1MASK   (0x000f)      /* Bits 0-3: ITU1 */
#define SH1_IPRC_ITU1SHIFT  (0)
#define SH1_IPRC_ITU0MASK   (0x00f0)      /* Bits 4-7: ITU0 */
#define SH1_IPRC_ITU0SHIFT  (4)
#define SH1_IPRC_DM23MASK   (0x0f00)      /* Bits 8-11: DMAC2,3 */
#define SH1_IPRC_DM23SHIFT  (8)
#define SH1_IPRC_DM01MASK   (0xf000)      /* Bits 12-15: DMAC0,1 */
#define SH1_IPRC_DM01SHIFT  (12)

#define SH1_IPRD_SCI0MASK   (0x000f)      /* Bits 0-3: SCI0 */
#define SH1_IPRD_SCI0SHIFT  (0)
#define SH1_IPRD_ITU4MASK   (0x00f0)      /* Bits 4-7: ITU4 */
#define SH1_IPRD_ITU4SHIFT  (4)
#define SH1_IPRD_ITU3MASK   (0x0f00)      /* Bits 8-11: ITU3 */
#define SH1_IPRD_ITU3SHIFT  (8)
#define SH1_IPRD_ITU2MASK   (0xf000)      /* Bits 12-15: ITU2 */
#define SH1_IPRD_ITU2SHIFT  (12)

#define SH1_IPRE_WDRFMASK   (0x00f0)      /* Bits 4-7: WDT, REF */
#define SH1_IPRE_WDRFSHIFT  (4)
#define SH1_IPRE_PRADMASK   (0x0f00)      /* Bits 8-11: PRT, A/D */
#define SH1_IPRE_PRADSHIFT  (8)
#define SH1_IPRE_SCI1MASK   (0xf000)      /* Bits 12-15: SCI1 */
#define SH1_IPRE_SCI1SHIFT  (12)

#define SH1_ICR_IRQ7S       (0x0001)      /* Bits 0: Interrupt on falling edge of IRQ7 input */
#define SH1_ICR_IRQ6S       (0x0002)      /* Bits 1: Interrupt on falling edge of IRQ6 input */
#define SH1_ICR_IRQ5S       (0x0004)      /* Bits 2: Interrupt on falling edge of IRQ5 input */
#define SH1_ICR_IRQ4S       (0x0008)      /* Bits 3: Interrupt on falling edge of IRQ4 input */
#define SH1_ICR_IRQ3S       (0x0010)      /* Bits 4: Interrupt on falling edge of IRQ3 input */
#define SH1_ICR_IRQ2S       (0x0020)      /* Bits 5: Interrupt on falling edge of IRQ2 input */
#define SH1_ICR_IRQ1S       (0x0040)      /* Bits 6: Interrupt on falling edge of IRQ1 input */
#define SH1_ICR_IRQ0S       (0x0080)      /* Bits 7: Interrupt on falling edge of IRQ0 input */
#define SH1_ICR_NMIE        (0x0100)      /* Bits 8: Interupt on rising edge of NMI input */
#define SH1_ICR_NMIL        (0x8000)      /* Bits 15: NMI input level high */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_SH_SRC_SH1_703X_H */














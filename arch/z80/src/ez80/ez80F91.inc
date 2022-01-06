
;***********************************************************************************
; arch/z80/src/ez80/ez80f91.h
;
; Licensed to the Apache Software Foundation (ASF) under one or more
; contributor license agreements.  See the NOTICE file distributed with
; this work for additional information regarding copyright ownership.  The
; ASF licenses this file to you under the Apache License, Version 2.0 (the
; "License"); you may not use this file except in compliance with the
; License.  You may obtain a copy of the License at
;
;   http://www.apache.org/licenses/LICENSE-2.0
;
; Unless required by applicable law or agreed to in writing, software
; distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
; WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
; License for the specific language governing permissions and limitations
; under the License. *
;***********************************************************************************

;***********************************************************************************
; Included Files
;***********************************************************************************

; Product ID Registers  ************************************************************

ZDI_ID_L		EQU	000h
ZDI_ID_H		EQU	001h
ZDI_ID_REV		EQU	002h

; Interrupt Registers  *************************************************************

INT_P0			EQU	010h
INT_P1			EQU	011h
INT_P2			EQU	012h
INT_P3			EQU	013h
INT_P4			EQU	014h
INT_P5			EQU	015h

; EMACC Registers  *****************************************************************

EMAC_TEST		EQU	020h  ; EMAC test register *
EMAC_CFG1		EQU	021h  ; EMAC configuration 1 register *
EMAC_CFG2		EQU	022h  ; EMAC configuration 2 register *
EMAC_CFG3		EQU	023h  ; EMAC configuration 3 register *
EMAC_CFG4		EQU	024h  ; EMAC configuration 4 register *
EMAC_STAD_0		EQU	025h  ; EMAC station address register 0 *
EMAC_STAD_1		EQU	026h  ; EMAC station address register 1 *
EMAC_STAD_2		EQU	027h  ; EMAC station address register 2 *
EMAC_STAD_3		EQU	028h  ; EMAC station address register 3 *
EMAC_STAD_4		EQU	029h  ; EMAC station address register 4 *
EMAC_STAD_5		EQU	02ah  ; EMAC station address register 5 *
EMAC_TPTV_L		EQU	02bh  ; Transit pause timer value (low) *
EMAC_TPTV_H		EQU	02ch  ; Transit pause timer value (high) *
EMAC_IPGT		EQU	02dh  ; EMAC Interpacket gap register *
EMAC_IPGR1		EQU	02eh  ; Non-back-to-back IPG register 1 *
EMAC_IPGR2		EQU	02fh  ; Non-back-to-back IPG register 2 *
EMAC_MAXF_L		EQU	030h  ; EMAC maximum frame length register (low) *
EMAC_MAXF_H		EQU	031h  ; EMAC maximum frame length register (high) *
EMAC_AFR		EQU	032h  ; EMAC address filter register *
EMAC_HTBL_0		EQU	033h  ; EMAC hash table register 0 *
EMAC_HTBL_1		EQU	034h  ; EMAC hash table register 1 *
EMAC_HTBL_2		EQU	035h  ; EMAC hash table register 2 *
EMAC_HTBL_3		EQU	036h  ; EMAC hash table register 3 *
EMAC_HTBL_4		EQU	037h  ; EMAC hash table register 4 *
EMAC_HTBL_5		EQU	038h  ; EMAC hash table register 5 *
EMAC_HTBL_6		EQU	039h  ; EMAC hash table register 6 *
EMAC_HTBL_7		EQU	03ah  ; EMAC hash table register 7 *
EMAC_MIIMGT		EQU	03bh  ; EMACS MII management register *
EMAC_CTLD_L		EQU	03ch  ; PHY configuration data register (low) *
EMAC_CTLD_H		EQU	03dh  ; PHY configuration data register (high) *
EMAC_RGAD		EQU	03eh  ; PHY address register *
EMAC_FIAD		EQU	03fh  ; PHY unit select register *
EMAC_PTMR		EQU	040h  ; EMAC transmit polling timer register *
EMAC_RST		EQU	041h  ; EMAC reset control register *
EMAC_TLBP_L		EQU	042h  ; EMAC transmit lower boundary pointer (low) *
EMAC_TLBP_H		EQU	043h  ; EMAC transmit lower boundary pointer (high) *
EMAC_BP_L		EQU	044h  ; EMAC boundary pointer register (low) *
EMAC_BP_H		EQU	045h  ; EMAC boundary pointer register (high) *
EMAC_BP_U		EQU	046h  ; EMAC boundary pointer register (upper byte) *
EMAC_RHBP_L		EQU	047h  ; EMAC receive high boundary pointer register (low) *
EMAC_RHBP_H		EQU	048h  ; EMAC receive high boundary pointer register (high) *
EMAC_RRP_L		EQU	049h  ; EMAC receive read pointer (low) *
EMAC_RRP_H		EQU	04ah  ; EMAC receive read pointer (high) *
EMAC_BUFSZ		EQU	04bh  ; EMAC buffer size register *
EMAC_IEN		EQU	04ch  ; EMAC interrupt enable register *
EMAC_ISTAT		EQU	04dh  ; EMAC interrupt status register *
EMAC_PRSD_L		EQU	04eh  ; PHY read status data register (low) *
EMAC_PRSD_H		EQU	04fh  ; PHY read status data register (high) *
EMAC_MIISTAT		EQU	050h  ; EMAC MII status register *
EMAC_RWP_L		EQU	051h  ; EMAC receive write pointer (low) *
EMAC_RWP_H		EQU	052h  ; EMAC receive write pointer (high) *
EMAC_TRP_L		EQU	053h  ; EMAC transmit read pointer (low) *
EMAC_TRP_H		EQU	054h  ; EMAC transmit read pointer (high) *
EMAC_BLKSLFT_L		EQU	055h  ; EMAC receive blocks left register (low) *
EMAC_BLKSLFT_H		EQU	056h  ; EMAC receive blocks left register (high) *
EMAC_FDATA_L		EQU	057h  ; EMAC FIFO data register (low) *
EMAC_FDATA_H		EQU	058h  ; EMAC FIFO data register (high) *
EMAC_FFLAGS		EQU	059h  ; EMAC FIFO flags register *

; PLL Registers  *******************************************************************

PLL_DIV_L		EQU	05ch
PLL_DIV_H		EQU	05dh
PLL_CTL0		EQU	05eh
PLL_CTL1		EQU	05fh

; Timer Registers  *****************************************************************

TMR0_CTL		EQU	060h        ; RW: Timer 0 control register *
TMR0_IER		EQU	061h        ; RW: Timer 0 interrupt enable register *
TMR0_IIR		EQU	062h        ; R : Timer 0 interrupt ID register *
TMR0_DRL		EQU	063h        ; R : Timer 0 data register (low) *
TMR0_DRH		EQU	064h        ; R : Timer 0 data register (high) *
TMR0_RRL		EQU	063h        ;  W: Timer 0 reload register (low) *
TMR0_RRH		EQU	064h        ;  W: Timer 0 reload register (high) *

TMR1_CTL		EQU	065h        ; RW: Timer 1 control register *
TMR1_IER		EQU	066h        ; RW: Timer 1 interrupt enable register *
TMR1_IIR		EQU	067h        ; R : Timer 1 interrupt ID register *
TMR1_DRL		EQU	068h        ; R : Timer 1 data register (low) *
TMR1_DRH		EQU	069h        ; R : Timer 1 data register (high) *
TMR1_RRL		EQU	068h        ;  W: Timer 1 reload register (low) *
TMR1_RRH		EQU	069h        ;  W: Timer 1 reload register (high) *
TMR1_CAPCTL		EQU	06ah        ; RW: Timer 1 input capture control register *
TMR1_CAPAL		EQU	06bh        ; R : Timer 1 capture input value A (low) *
TMR1_CAPAH		EQU	06ch        ; R : Timer 1 capture input value A (high) *
TMR1_CAPBL		EQU	06dh        ; R : Timer 1 capture input value B (low) *
TMR1_CAPBH		EQU	06eh

TMR2_CTL		EQU	06fh        ; RW: Timer 2 control register *
TMR2_IER		EQU	070h        ; RW: Timer 2 interrupt enable register *
TMR2_IIR		EQU	071h        ; R : Timer 2 interrupt ID register *
TMR2_DRL		EQU	072h        ; R : Timer 2 data register (low) *
TMR2_DRH		EQU	073h        ; R : Timer 2 data register (high) *
TMR2_RRL		EQU	072h        ;  W: Timer 2 reload register (low) *
TMR2_RRH		EQU	073h        ;  W: Timer 2 reload register (high) *

TMR3_CTL		EQU	074h        ; RW: Timer 3 control register *
TMR3_IER		EQU	075h        ; RW: Timer 3 interrupt enable register *
TMR3_IIR		EQU	076h        ; R : Timer 3 interrupt ID register *
TMR3_DRL		EQU	077h        ; R : Timer 3 data register (low) *
TMR3_DRH		EQU	078h        ; R : Timer 3 data register (high) *
TMR3_RRL		EQU	077h        ;  W: Timer 3 reload register (low) *
TMR3_RRH		EQU	078h        ;  W: Timer 3 reload register (high) *
TMR3_CAPCTL		EQU	07bh        ; RW: Timer 3 input capture control register *
TMR3_CAPAL		EQU	07ch        ; R : Timer 3 capture input value A (low) *
TMR3_CAPAH		EQU	07dh        ; R : Timer 3 capture input value A (high) *
TMR3_CAPBL		EQU	07eh        ; R : Timer 3 capture input value B (low) *
TMR3_CAPBH		EQU	07fh        ; R : Timer 3 capture input value B (high) *
TMR3_OCCTL1		EQU	080h        ; RW: Timer 3 output compare control register1 *
TMR3_OCCTL2		EQU	081h        ; RW: Timer 3 output compare control register2 *
TMR3_OC0L		EQU	082h        ; RW: Timer 3 output compare value 0 (low) *
TMR3_OC0H		EQU	083h        ; RW: Timer 3 output compare value 0 (high) *
TMR3_OC1L		EQU	084h        ; RW: Timer 3 output compare value 1 (low) *
TMR3_OC1H		EQU	085h        ; RW: Timer 3 output compare value 1 (high) *
TMR3_OC2L		EQU	086h        ; RW: Timer 3 output compare value 2 (low) *
TMR3_OC2H		EQU	087h        ; RW: Timer 3 output compare value 2 (high) *
TMR3_OC3L		EQU	088h        ; RW: Timer 3 output compare value 3 (low) *
TMR3_OC3H		EQU	089h        ; RW: Timer 3 output compare value 3 (high) *

; TMR0/1/2/3 CTL Register Bit Definitions ******************************************

TMRCTL_BRKSTOP		EQU	080h        ; Bit 7: Stop timer for debug operation *
TMRCTL_CLKSEL		EQU	060h        ; Bits 6-5: Timer source *
TMRCLKSEL_SYSCLK	EQU	000h       ;   00: System clock divided by prescaler *
TMRCLKSEL_RTC		EQU	020h        ;   01: Real time clock input *
TMRCLKSEL_ECF		EQU	040h        ;   10: Event count input, falling edge *
TMRCLKSEL_ECR		EQU	060h        ;   11: Event count input, rising edge *
TMRCTL_CLKDIV		EQU	018h        ; Bits 3-4: System clock divider *
TMRCLKDIV_4		EQU	000h        ;   00:   4 *
TMRCLKDIV_16		EQU	008h        ;   01:  16 *
TMRCLKDIV_64		EQU	010h        ;   10:  64 *
TMRCLKDIV_256		EQU	018h        ;   11: 256 *
TMRCTL_TIMCONT		EQU	004h        ; Bit 2: Continuous mode *
TMRCTL_RLD		EQU	002h        ; Bit 1: Force reload *
TMRCTL_TIMEN		EQU	001h        ; Bit 0: Programmable reload timer enabled *

; TMR0/1/2/3 IER Register Bit Definitions ******************************************

                                           ; Bit 7: Reserved *
TMRIER_OC3EN		EQU	040h        ; Bit 6: TMR3 OC3 enabled *
TMRIER_OC2EN		EQU	020h        ; Bit 5: TMR3 OC2 enabled *
TMRIER_OC1EN		EQU	010h        ; Bit 4: TMR3 OC1 enabled *
TMRIER_OC0EN		EQU	008h        ; Bit 3: TMR3 OC0 enabled *
TMRIER_ICBEN		EQU	004h        ; Bit 2: TMR1/3 capture pin enabled *
TMRIER_ICAEN		EQU	002h        ; Bit 1: TMR1/3 capture pin enabled *
TMRIER_EOCEN		EQU	001h        ; Bit 0: End of count interrupt enabled *

; TMR0/1/2/3 IER Register Bit Definitions ******************************************

                                           ; Bit 7: Reserved *
TMRIIR_OC3		EQU	040h        ; Bit 6: TMR3 OC3 *
TMRIIR_OC2		EQU	020h        ; Bit 5: TMR3 OC2 *
TMRIIR_OC1		EQU	010h        ; Bit 4: TMR3 OC1 *
TMRIIR_OC0		EQU	008h        ; Bit 3: TMR3 OC0 *
TMRIIR_ICB		EQU	004h        ; Bit 2: TMR1/3 capture pin *
TMRIIR_ICA		EQU	002h        ; Bit 1: TMR1/3 capture pin *
TMRIIR_EOC		EQU	001h        ; Bit 0: End of count interrupt *

; PWM Registers ********************************************************************

PWM_CTL1		EQU	079h
PWM_CTL2		EQU	07ah
PWM_CTL3		EQU	07bh
PWM0R_L			EQU	07ch
PWM0R_H			EQU	07dh
PWM1R_L			EQU	07eh
PWM1R_H			EQU	07fh
PWM2R_L			EQU	080h
PWM2R_H			EQU	081h
PWM3R_L			EQU	082h
PWM3R_H			EQU	083h
PWM0F_L			EQU	084h
PWM0F_H			EQU	085h
PWM1F_L			EQU	086h
PWM1F_H			EQU	087h
PWM2F_L			EQU	088h
PWM2F_H			EQU	089h
PWM3F_L			EQU	08ah
PWM3F_H			EQU	08bh

; WDT Registers ********************************************************************

WDT_CTL			EQU	093h
WDT_RR			EQU	094h

; GPIO Registers *******************************************************************

PA_DR			EQU	096h
PA_DDR			EQU	097h
PA_ALT0			EQU	0a6h
PA_ALT1			EQU	098h
PA_ALT2			EQU	099h
PB_DR			EQU	09ah
PB_DDR			EQU	09bh
PB_ALT0			EQU	0a7h
PB_ALT1			EQU	09ch
PB_ALT2			EQU	09dh
PC_DR			EQU	09eh
PC_DDR			EQU	09fh
PC_ALT0			EQU	0ceh
PC_ALT1			EQU	0a0h
PC_ALT2			EQU	0a1h
PD_DR			EQU	0a2h
PD_DDR			EQU	0a3h
PD_ALT0			EQU	0cfh
PD_ALT1			EQU	0a4h
PD_ALT2			EQU	0a5h

; CS Registers *********************************************************************

CS0_LBR			EQU	0a8h
CS0_UBR			EQU	0a9h
CS0_CTL			EQU	0aah
CS1_LBR			EQU	0abh
CS1_UBR			EQU	0ach
CS1_CTL			EQU	0adh
CS2_LBR			EQU	0aeh
CS2_UBR			EQU	0afh
CS2_CTL			EQU	0b0h
CS3_LBR			EQU	0b1h
CS3_UBR			EQU	0b2h
CS3_CTL			EQU	0b3h

; RAMCTL reggisters ****************************************************************

RAM_CTL			EQU	0b4h
RAM_CTL0		EQU	0b4h
RAM_ADDR_U		EQU	0b5h
MBIST_GPR		EQU	0b6h
MBIST_EMR		EQU	0b7h

; RAMCTL bit definitions ***********************************************************

RAMCTL_ERAMEN		EQU	(1 << 6) ; Bit 7: 1=On chip EMAC SRAM is enabled *
RAMCTL_GPRAMEN		EQU	(1 << 7) ; Bit 7: 1=On chip GP SRAM is enabled *

; SPI Registers ********************************************************************

SPI_BRG_L		EQU	0b8h
SPI_BRG_H		EQU	0b9h
SPI_CTL			EQU	0bah
SPI_SR			EQU	0bbh
SPI_RBR			EQU	0bch
SPI_TSR			EQU	0bch

; UART Register Offsets ************************************************************

                                           ; DLAB=0: *
UART_THR		EQU	000h        ;    W: UART Transmit holding register *
UART_RBR		EQU	000h        ;   R : UART Receive buffer register *
UART_IER		EQU	001h        ;   RW: UART Interrupt enable register *
                                           ; DLAB=1: *
UART_BRG		EQU	000h        ;   RW: UART Baud rate generator register *
UART_BRGL		EQU	000h        ;   RW: UART Baud rate generator register (low) *
UART_BRGH		EQU	001h        ;   RW: UART Baud rate generator register (high) *
                                           ; DLAB=N/A: *
UART_IIR		EQU	002h        ;   R : UART Interrupt identification register *
UART_FCTL		EQU	002h        ;    W: UART FIFO control register *
UART_LCTL		EQU	003h        ;   RW: UART Line control register *
UART_MCTL		EQU	004h        ;   RW: UART Modem control register *
UART_LSR		EQU	005h        ;   R : UART Line status register *
UART_MSR		EQU	006h        ;   R : UART Modem status register *
UART_SPR		EQU	007h        ;   RW: UART Scratchpad register *

; UART0/1 Base Register Addresses **************************************************

UART0_BASE		EQU	0c0h
UART1_BASE		EQU	0d0h

; UART0/1 Register Addresses *******************************************************

UART0_THR		EQU	UART0_BASE + UART_THR
UART0_RBR		EQU	UART0_BASE + UART_RBR
UART0_IER		EQU	UART0_BASE + UART_IER
UART0_BRG		EQU	UART0_BASE + UART_BRG
UART0_BRGL		EQU	UART0_BASE + UART_BRGL
UART0_BRGH		EQU	UART0_BASE + UART_BRGH
UART0_IIR		EQU	UART0_BASE + UART_IIR
UART0_FCTL		EQU	UART0_BASE + UART_FCTL
UART0_LCTL		EQU	UART0_BASE + UART_LCTL
UART0_MCTL		EQU	UART0_BASE + UART_MCTL
UART0_LSR		EQU	UART0_BASE + UART_LSR
UART0_MSR		EQU	UART0_BASE + UART_MSR
UART0_SPR		EQU	UART0_BASE + UART_SPR

UART1_THR		EQU	UART1_BASE + UART_THR
UART1_RBR		EQU	UART1_BASE + UART_RBR
UART1_IER		EQU	UART1_BASE + UART_IER
UART1_BRG		EQU	UART1_BASE + UART_BRG
UART1_BRGL		EQU	UART1_BASE + UART_BRGL
UART1_BRGH		EQU	UART1_BASE + UART_BRGH
UART1_IIR		EQU	UART1_BASE + UART_IIR
UART1_FCTL		EQU	UART1_BASE + UART_FCTL
UART1_LCTL		EQU	UART1_BASE + UART_LCTL
UART1_MCTL		EQU	UART1_BASE + UART_MCTL
UART1_LSR		EQU	UART1_BASE + UART_LSR
UART1_MSR		EQU	UART1_BASE + UART_MSR
UART1_SPR		EQU	UART1_BASE + UART_SPR

; UART0/1 IER register bits ********************************************************

UARTEIR_INTMASK		EQU	01fh         ; Bits 5-7: Reserved *
UARTEIR_TCIE		EQU	010h         ; Bit 4: Transmission complete interrupt *
UARTEIR_MIIE		EQU	008h         ; Bit 3: Modem status input interrupt *
UARTEIR_LSIE		EQU	004h         ; Bit 2: Line status interrupt *
UARTEIR_TIE		EQU	002h         ; Bit 1: Transmit interrupt *
UARTEIR_RIE		EQU	001h         ; Bit 0: Receive interrupt *

; UART0/1 IIR register bits ********************************************************

UARTIIR_FSTS		EQU	080h         ; Bit 7: FIFO enable *
                                            ; Bits 4-6: Reserved *
UARTIIR_INSTS		EQU	00eh         ; Bits 1-3: Interrupt status code *
UARTINSTS_CTO		EQU	00ch         ;   110: Character timeout *
UARTINSTS_TC		EQU	00ah         ;   101: Transmission complete *
UARTINSTS_RLS		EQU	006h         ;   011: Receiver line status *
UARTINSTS_RDR		EQU	004h         ;   010: Receive data ready or trigger level *
UARTINSTS_TBE		EQU	002h         ;   001: Transmisson buffer empty *
UARTINSTS_MS		EQU	000h         ;   000: Modem status *
UARTIIR_INTBIT		EQU	001h         ; Bit 0: Active interrupt source *
UARTIIR_CAUSEMASK	EQU	00fh

; UART0/1 FCTL register bits *******************************************************

UARTFCTL_TRIG		EQU	0c0h         ; Bits 6-7: UART receive FIFO trigger level *
UARTTRIG_1		EQU	000h         ;   00: Receive FIFO trigger level=1 *
UARTTRIG_4		EQU	040h         ;   01: Receive FIFO trigger level=4 *
UARTTRIG_8		EQU	080h         ;   10: Receive FIFO trigger level=8 *
UARTTRIG_14		EQU	0c0h         ;   11: Receive FIFO trigger level=14 *
                                            ; Bit 3-5: Reserved *
UARTFCTL_CLRTXF		EQU	004h         ; Bit 2: Transmit enable *
UARTFCTL_CLRRXF		EQU	002h         ; Bit 1: Receive enable *
UARTFCTL_FIFOEN		EQU	001h         ; Bit 0: Enable receive/transmit FIFOs *

; UART0/1 LCTL register bits *******************************************************

UARTLCTL_DLAB		EQU	080h         ; Bit 7: Enable access to baud rate generator *
UARTLCTL_SB		EQU	040h         ; Bit 6: Send break *
UARTLCTL_FPE		EQU	020h         ; Bit 5: Force parity error *
UARTLCTL_EPS		EQU	010h         ; Bit 4: Even parity select *
UARTLCTL_PEN		EQU	008h         ; Bit 3: Parity enable *
UARTLCTL_2STOP		EQU	004h         ; Bit 2: 2 stop bits *
UARTLCTL_CHAR		EQU	003h         ; Bits 0-2: Number of data bits *
UARTCHAR_5BITS		EQU	000h         ;   00: 5 data bits *
UARTCHAR_6BITS		EQU	001h         ;   01: 6 data bits *
UARTCHAR_7BITS		EQU	002h         ;   10: 7 data bits *
UARTCHAR_8BITS		EQU	003h         ;   11: 8 data bits *

UARTLCTL_MASK		EQU	03fh

; UART0/1 MCTL register bits *******************************************************

                                            ; Bit 7: Reserved *
UARTMCTL_POLARITY	EQU	040h         ; Bit 6: Invert polarity of RxD and TxD *
UARTMCTL_MDM		EQU	020h         ; Bit 5: Multi-drop mode enable *
UARTMCTL_LOOP		EQU	010h         ; Bit 4: Loopback mode enable *
UARTMCTL_OUT2		EQU	008h         ; Bit 3: (loopback mode only) *
UARTMCTL_OUT1		EQU	004h         ; Bit 2: (loopback mode only) *
UARTMCTL_RTS		EQU	002h         ; Bit 1: Request to send *
UARTMCTL_DTR		EQU	001h         ; Bit 0: Data termnal read *

; UART0/1 LSR register bits ********************************************************

UARTLSR_ERR		EQU	080h         ; Bit 7: Error detected in FIFO *
UARTLSR_TEMT		EQU	040h         ; Bit 6: Transmit FIFO empty and idle *
UARTLSR_THRE		EQU	020h         ; Bit 5: Transmit FIFO empty *
UARTLSR_BI		EQU	010h         ; Bit 4: Break on input *
UARTLSR_FE		EQU	008h         ; Bit 3: Framing error *
UARTLSR_PE		EQU	004h         ; Bit 2: Parity error *
UARTLSR_OE		EQU	002h         ; Bit 1: Overrun error *
UARTLSR_DR		EQU	001h         ; Bit 0: Data ready *

; UART0/1 MSR register bits ********************************************************

UARTMSR_DCD		EQU	080h         ; Bit 7: Data carrier detect *
UARTMSR_RI		EQU	040h         ; Bit 6: Ring indicator *
UARTMSR_DSR		EQU	020h         ; Bit 5: Data set ready *
UARTMSR_CTS		EQU	010h         ; Bit 4: Clear to send *
UARTMSR_DDCD		EQU	008h         ; Bit 3: Delta on DCD input *
UARTMSR_TERI		EQU	004h         ; Bit 2: Trailing edge change on RI *
UARTMSR_DDSR		EQU	002h         ; Bit 1: Delta on DSR input *
UARTMSR_DCTS		EQU	001h         ; Bit 0: Delta on CTS input *

; IR Registers  ********************************************************************

IR_CTL			EQU	0bfh

; I2C Registers  *******************************************************************

I2C_SAR			EQU	0c8h
I2C_XSAR		EQU	0c9h
I2C_DR			EQU	0cah
I2C_CTL			EQU	0cbh
I2C_SR			EQU	0cch
I2C_CCR			EQU	0cdh
I2C_SRR			EQU	0ceh

; CLK Registers  *******************************************************************

CLK_PPD1		EQU	0dbh
CLK_PPD2		EQU	0dch

; RTC Registers  *******************************************************************

RTC_SEC			EQU	0e0h
RTC_MIN			EQU	0e1h
RTC_HRS			EQU	0e2h
RTC_DOW			EQU	0e3h
RTC_DOM			EQU	0e4h
RTC_MON			EQU	0e5h
RTC_YR			EQU	0e6h
RTC_CEN			EQU	0e7h
RTC_ASEC		EQU	0e8h
RTC_AMIN		EQU	0e9h
RTC_AHRS		EQU	0eah
RTC_ADOW		EQU	0ebh
RTC_ACTRL		EQU	0ech
RTC_CTRL		EQU	0edh

; CSBMC Registers  *****************************************************************

CS0_BMC			EQU	0f0h
CS1_BMC			EQU	0f1h
CS2_BMC			EQU	0f2h
CS3_BMC			EQU	0f3h

; FLASH Registers  *****************************************************************

FLASH_KEY		EQU	0f5h
FLASH_DATA		EQU	0f6h
FLASH_ADDR_U		EQU	0f7h
FLASH_CTRL		EQU	0f8h
FLASH_FDIV		EQU	0f9h
FLASH_PROT		EQU	0fah
FLASH_INTC		EQU	0fbh
FLASH_IRQ		EQU	FLASH_INTC	; compat. with ZDS-II
FLASH_PAGE		EQU	0fch
FLASH_ROW		EQU	0fdh
FLASH_COL		EQU	0feh
FLASH_PGCTL		EQU	0ffh


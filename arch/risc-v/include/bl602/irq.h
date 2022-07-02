/****************************************************************************
 * arch/risc-v/include/bl602/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_BL602_IRQ_H
#define __ARCH_RISCV_INCLUDE_BL602_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CLINT Base Address */

#define CLIC_TIMER_ENABLE_ADDRESS (0x02800407)

/* Map RISC-V exception code to NuttX IRQ */

#define BL602_IRQ_NUM_BASE (16)

#define BL602_IRQ_BMX_ERR \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 0) /* BMX Error Interrupt */
#define BL602_IRQ_BMX_TO \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 1) /* BMX Timeout Interrupt */
#define BL602_IRQ_L1C_BMX_ERR \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 2) /* L1C BMX Error Interrupt */
#define BL602_IRQ_L1C_BMX_TO \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 3) /* L1C BMX Timeout Interrupt */
#define BL602_IRQ_SEC_BMX_ERR \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 4) /* SEC BMX Error Interrupt */
#define BL602_IRQ_RF_TOP_INT0 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 5) /* RF_TOP_INT0 Interrupt */
#define BL602_IRQ_RF_TOP_INT1 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 6) /* RF_TOP_INT1 Interrupt */
#define BL602_IRQ_SDIO \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 7) /* SDIO Interrupt */
#define BL602_IRQ_DMA_BMX_ERR \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 8) /* DMA BMX Error Interrupt */
#define BL602_IRQ_SEC_GMAC \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 9) /* SEC_ENG_GMAC_INT Interrupt */
#define BL602_IRQ_SEC_CDET \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 10) /* SEC_ENG_CDET_INT Interrupt */
#define BL602_IRQ_SEC_PKA \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 11) /* SEC_ENG_PKA_INT  Interrupt */
#define BL602_IRQ_SEC_TRNG \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 12) /* SEC_ENG_TRNG_INT Interrupt */
#define BL602_IRQ_SEC_AES \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 13) /* SEC_ENG_AES_INT  Interrupt */
#define BL602_IRQ_SEC_SHA \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 14) /* SEC_ENG_SHA_INT  Interrupt */
#define BL602_IRQ_DMA_ALL \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 15) /* DMA ALL Interrupt */
#define BL602_IRQ_RESERVED0 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 16) /* RESERVED Interrupt */
#define BL602_IRQ_RESERVED1 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 17) /* RESERVED Interrupt */
#define BL602_IRQ_RESERVED2 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 18) /* RESERVED Interrupt */
#define BL602_IRQ_IRTX_IRQn \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 19) /* IR TX Interrupt */
#define BL602_IRQ_IRRX_IRQn \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 20) /* IR RX Interrupt */
#define BL602_IRQ_RESERVED3 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 21) /* RESERVED  Interrupt */
#define BL602_IRQ_RESERVED4 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 22) /* RESERVED  Interrupt */
#define BL602_IRQ_SF_CTRL \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 23) /* SF_CTRL   Interrupt */
#define BL602_IRQ_RESERVED5 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 24) /* RESERVED  Interrupt */
#define BL602_IRQ_GPADC_DMA \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 25) /* GPADC_DMA Interrupt */
#define BL602_IRQ_EFUSE \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 26) /* Efuse Interrupt */
#define BL602_IRQ_SPI \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 27) /* SPI   Interrupt */
#define BL602_IRQ_RESERVED6 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 28) /* RESERVED Interrupt */
#define BL602_IRQ_UART0 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 29) /* UART  Interrupt */
#define BL602_IRQ_UART1 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 30) /* UART1 Interrupt */
#define BL602_IRQ_RESERVED7 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 31) /* RESERVED Interrupt */
#define BL602_IRQ_I2C \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 32) /* I2C   Interrupt */
#define BL602_IRQ_RESERVED8 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 33) /* RESERVED Interrupt */
#define BL602_IRQ_PWM \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 34) /* PWM   Interrupt */
#define BL602_IRQ_RESERVED9 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 35) /* RESERVED Interrupt */
#define BL602_IRQ_TIMER_CH0 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 36) /* Timer Channel 0 Interrupt */
#define BL602_IRQ_TIMER_CH1 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 37) /* Timer Channel 1 Interrupt */
#define BL602_IRQ_TIMER_WDT \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 38) /* Timer Watch Dog Interrupt */
#define BL602_IRQ_RESERVED10 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 39) /* RESERVED Interrupt */
#define BL602_IRQ_RESERVED11 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 40) /* RESERVED Interrupt */
#define BL602_IRQ_RESERVED12 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 41) /* RESERVED Interrupt */
#define BL602_IRQ_RESERVED13 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 42) /* RESERVED Interrupt */
#define BL602_IRQ_RESERVED14 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 43) /* RESERVED Interrupt */
#define BL602_IRQ_GPIO_INT0 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 44) /* RESERVED Interrupt */
#define BL602_IRQ_RESERVED16 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 45) /* RESERVED Interrupt */
#define BL602_IRQ_RESERVED17 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 46) /* RESERVED Interrupt */
#define BL602_IRQ_RESERVED18 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 47) /* RESERVED Interrupt */
#define BL602_IRQ_RESERVED19 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 48) /* RESERVED Interrupt */
#define BL602_IRQ_RESERVED20 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 49) /* RESERVED Interrupt */
#define BL602_IRQ_PDS_WAKEUP \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 50) /* PDS Wakeup Interrupt */
#define BL602_IRQ_HBN_OUT0 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 51) /* Hibernate out 0 Interrupt */
#define BL602_IRQ_HBN_OUT1 \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 52) /* Hibernate out 1 Interrupt */
#define BL602_IRQ_BOR \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 53) /* BOR Interrupt */
#define BL602_IRQ_WIFI \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 54) /* WIFI To CPU Interrupt */
#define BL602_IRQ_BZ_PHY \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 55) /* RESERVED Interrupt */
#define BL602_IRQ_BLE \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 56) /* RESERVED Interrupt */
#define BL602_IRQ_MAC_TXRX_TIMER \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + \
   57) /* mac_int_tx_rx_timer Interrupt */
#define BL602_IRQ_MAC_TXRX_MISC \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + \
   58) /* mac_int_tx_rx_misc Interrupt */
#define BL602_IRQ_MAC_RX_TRG \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + \
   59) /* mac_int_rx_trigger Interrupt */
#define BL602_IRQ_MAC_TX_TRG \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + \
   60) /* mac_int_tx_trigger Interrupt */
#define BL602_IRQ_MAC_GEN \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 61) /* mac_int_gen Interrupt */
#define BL602_IRQ_MAC_PORT_TRG \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + \
   62) /* mac_int_port_trigger Interrupt */
#define BL602_IRQ_WIFI_IPC_PUBLIC \
  (RISCV_IRQ_ASYNC + BL602_IRQ_NUM_BASE + 63) /* wifi IPC public Interrupt */

/* Total number of IRQs */

#define NR_IRQS (64 + 16 + 16)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

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

EXTERN irqstate_t up_irq_enable(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_INCLUDE_BL602_IRQ_H */


/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_dma.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_DMA_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_DMA_H

#include <nuttx/config.h>
#include <stdbool.h>
#include <stdint.h>

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
 * Pre-processor Macros
 ****************************************************************************/

/* DMA max data length */

#define ESPRESSIF_DMA_DATALEN_MAX (0x1000 - 4)

/* DMA max buffer length */

#define ESPRESSIF_DMA_BUFLEN_MAX  ESPRESSIF_DMA_DATALEN_MAX

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum esp_dma_periph_e
{
  ESPRESSIF_DMA_PERIPH_M2M,
  ESPRESSIF_DMA_PERIPH_UHCI,
  ESPRESSIF_DMA_PERIPH_SPI,
  ESPRESSIF_DMA_PERIPH_I2S,
  ESPRESSIF_DMA_PERIPH_AES,
  ESPRESSIF_DMA_PERIPH_SHA,
  ESPRESSIF_DMA_PERIPH_ADC,
  ESPRESSIF_DMA_PERIPH_DAC,
  ESPRESSIF_DMA_PERIPH_LCD,
  ESPRESSIF_DMA_PERIPH_CAM,
  ESPRESSIF_DMA_PERIPH_RMT,
  ESPRESSIF_DMA_PERIPH_PARLIO,
};

/* DMA descriptor type */

struct esp_dmadesc_s
{
  uint32_t ctrl;                /* DMA control block */
  const uint8_t *pbuf;          /* DMA TX/RX buffer address */
  struct esp_dmadesc_s *next;   /* Next DMA descriptor address */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_dma_request
 *
 * Description:
 *   Request DMA channel and config it with given parameters.
 *
 * Input Parameters:
 *   periph  - Peripheral for which the DMA channel request was made
 *   tx_prio - Interrupt priority
 *   rx_prio - Interrupt flags
 *
 * Returned Value:
 *   DMA channel number (>=0) if success or -1 if fail.
 *
 ****************************************************************************/

int32_t esp_dma_request(enum esp_dma_periph_e periph,
                        uint32_t tx_prio,
                        uint32_t rx_prio,
                        bool burst_en);

/****************************************************************************
 * Name: esp_dma_setup
 *
 * Description:
 *   Set up DMA descriptor with given parameters.
 *
 * Input Parameters:
 *   chan    - DMA channel
 *   tx      - true: TX mode; false: RX mode
 *   dmadesc - DMA descriptor pointer
 *   num     - DMA descriptor number
 *   pbuf    - Buffer pointer
 *   len     - Buffer length by byte
 *
 * Returned Value:
 *   Bind pbuf data bytes.
 *
 ****************************************************************************/

uint32_t esp_dma_setup(int chan, bool tx,
                       struct esp_dmadesc_s *dmadesc, uint32_t num,
                       uint8_t *pbuf, uint32_t len);

/****************************************************************************
 * Name: esp_dma_load
 *
 * Description:
 *   Load the address of the first DMA descriptor of an already bound
 *   inlink/outlink to the corresponding GDMA_<IN/OUT>LINK_ADDR_CHn register
 *
 * Input Parameters:
 *   dmadesc - Pointer of the previously bound inlink/outlink
 *   chan    - DMA channel of the receiver/transmitter
 *   tx      - true: TX mode (transmitter); false: RX mode (receiver)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_dma_load(struct esp_dmadesc_s *dmadesc, int chan, bool tx);

/****************************************************************************
 * Name: esp_dma_enable
 *
 * Description:
 *   Enable DMA channel transmission.
 *
 * Input Parameters:
 *   chan - DMA channel
 *   tx   - true: TX mode; false: RX mode
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_dma_enable(int chan, bool tx);

/****************************************************************************
 * Name: esp_dma_disable
 *
 * Description:
 *   Disable DMA channel transmission.
 *
 * Input Parameters:
 *   chan - DMA channel
 *   tx   - true: TX mode; false: RX mode
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_dma_disable(int chan, bool tx);

/****************************************************************************
 * Name: esp_dma_wait_idle
 *
 * Description:
 *   Wait until transmission ends.
 *
 * Input Parameters:
 *   chan - DMA channel
 *   tx   - true: TX mode; false: RX mode
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_dma_wait_idle(int chan, bool tx);

/****************************************************************************
 * Name: esp_dma_init
 *
 * Description:
 *   Initialize DMA driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_dma_init(void);

/****************************************************************************
 * Name: esp_dma_deinit
 *
 * Description:
 *   Deinitialize DMA driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_dma_deinit(void);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_DMA_H */

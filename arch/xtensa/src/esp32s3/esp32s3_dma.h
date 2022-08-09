/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_dma.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_DMA_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

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

#define ESP32S3_DMA_DATALEN_MAX       (0x1000 - 4)

/* DMA max buffer length */

#define ESP32S3_DMA_BUFLEN_MAX        ESP32S3_DMA_DATALEN_MAX

/* DMA channel number */

#define ESP32S3_DMA_CHAN_MAX          (3)

/* DMA RX MAX priority */

#define ESP32S3_DMA_RX_PRIO_MAX       (15)

/* DMA TX MAX priority */

#define ESP32S3_DMA_TX_PRIO_MAX       (15)

/* DMA descriptor */

#define ESP32S3_DMA_CTRL_OWN          (1 << 31)   /* Owned by DMA */
#define ESP32S3_DMA_CTRL_EOF          (1 << 30)   /* End of frame */
#define ESP32S3_DMA_CTRL_ERREOF       (1 << 28)   /* Received data error */
#define ESP32S3_DMA_CTRL_DATALEN_S    (12)        /* Data length shift */
#define ESP32S3_DMA_CTRL_DATALEN_V    (0xfff)     /* Data length value */
#define ESP32S3_DMA_CTRL_BUFLEN_S     (0)         /* Buffer length shift */
#define ESP32S3_DMA_CTRL_BUFLEN_V     (0xfff)     /* Buffer length value */

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum esp32s3_dma_periph_e
{
  ESP32S3_DMA_PERIPH_MEM    = -1,
  ESP32S3_DMA_PERIPH_SPI2   = 0,
  ESP32S3_DMA_PERIPH_SPI3   = 1,
  ESP32S3_DMA_PERIPH_UCHI0  = 2,
  ESP32S3_DMA_PERIPH_I2S0   = 3,
  ESP32S3_DMA_PERIPH_I2S1   = 4,
  ESP32S3_DMA_PERIPH_LCDCAM = 5,
  ESP32S3_DMA_PERIPH_AES    = 6,
  ESP32S3_DMA_PERIPH_SHA    = 7,
  ESP32S3_DMA_PERIPH_ADC    = 8,
  ESP32S3_DMA_PERIPH_RMT    = 9,
  ESP32S3_DMA_PERIPH_NUM,
};

/* DMA descriptor type */

struct esp32s3_dmadesc_s
{
  uint32_t ctrl;                    /* DMA control block */
  const uint8_t *pbuf;              /* DMA TX/RX buffer address */
  struct esp32s3_dmadesc_s *next;   /* Next DMA descriptor address */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_dma_request
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

int32_t esp32s3_dma_request(enum esp32s3_dma_periph_e periph,
                            uint32_t tx_prio,
                            uint32_t rx_prio,
                            bool burst_en);

/****************************************************************************
 * Name: esp32s3_dma_setup
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

uint32_t esp32s3_dma_setup(int chan, bool tx,
                           struct esp32s3_dmadesc_s *dmadesc, uint32_t num,
                           uint8_t *pbuf, uint32_t len);

/****************************************************************************
 * Name: esp32s3_dma_enable
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

void esp32s3_dma_enable(int chan, bool tx);

/****************************************************************************
 * Name: esp32s3_dma_disable
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

void esp32s3_dma_disable(int chan, bool tx);

/****************************************************************************
 * Name: esp32s3_dma_wait_idle
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

void esp32s3_dma_wait_idle(int chan, bool tx);

/****************************************************************************
 * Name: esp32s3_dma_init
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

void esp32s3_dma_init(void);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_DMA_H */

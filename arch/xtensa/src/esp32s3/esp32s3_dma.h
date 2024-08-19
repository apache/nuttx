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

/* The offset between different input/output GDMA channels' registers */

#define GDMA_REG_OFFSET  (DMA_OUT_CONF0_CH1_REG - DMA_OUT_CONF0_CH0_REG)

#define SET_GDMA_CH_REG(_r, _ch, _v)    putreg32((_v), (_r) + (_ch) * GDMA_REG_OFFSET)
#define GET_GDMA_CH_REG(_r, _ch)        getreg32((_r) + (_ch) * GDMA_REG_OFFSET)

#define SET_GDMA_CH_BITS(_r, _ch, _b)   modifyreg32((_r) + (_ch) * GDMA_REG_OFFSET, 0, (_b))
#define CLR_GDMA_CH_BITS(_r, _ch, _b)   modifyreg32((_r) + (_ch) * GDMA_REG_OFFSET, (_b), 0)

/* Maximum size of the buffer that can be attached to DMA descriptor  */

#define ESP32S3_DMA_BUFFER_MAX_SIZE       (0x1000 - 1)

/* DMA max data length, and aligned to 4Bytes */

#define ESP32S3_DMA_BUFLEN_MAX_4B_ALIGNED (0x1000 - 4)

/* DMA max buffer length */

#define ESP32S3_DMA_BUFLEN_MAX        ESP32S3_DMA_BUFFER_MAX_SIZE

/* DMA channel number */

#define ESP32S3_DMA_CHAN_MAX          (5)

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

enum esp32s3_dma_ext_memblk_e
{
  ESP32S3_DMA_EXT_MEMBLK_16B = 0,
  ESP32S3_DMA_EXT_MEMBLK_32B = 1,
  ESP32S3_DMA_EXT_MEMBLK_64B = 2
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
 * Name: esp32s3_dma_release
 *
 * Description:
 *   Release DMA channel from peripheral.
 *
 * Input Parameters:
 *   chan - Peripheral for which the DMA channel request was made
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_dma_release(int chan);

/****************************************************************************
 * Name: esp32s3_dma_setup
 *
 * Description:
 *   Initialize the DMA inlink/outlink (linked list) and bind the target
 *   buffer to its DMA descriptors.
 *
 * Input Parameters:
 *   dmadesc - Pointer to the DMA descriptors
 *   num     - Number of DMA descriptors
 *   pbuf    - RX/TX buffer pointer
 *   len     - RX/TX buffer length
 *   tx      - true: TX mode (transmitter); false: RX mode (receiver)
 *   chan    - DMA channel of the receiver/transmitter
 *
 * Returned Value:
 *   Bound pbuf data bytes
 *
 ****************************************************************************/

uint32_t esp32s3_dma_setup(struct esp32s3_dmadesc_s *dmadesc, uint32_t num,
                           uint8_t *pbuf, uint32_t len, bool tx, int chan);

/****************************************************************************
 * Name: esp32s3_dma_load
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

void esp32s3_dma_load(struct esp32s3_dmadesc_s *dmadesc, int chan, bool tx);

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
 * Name: esp32s3_dma_set_ext_memblk
 *
 * Description:
 *   Configure DMA external memory block size.
 *
 * Input Parameters:
 *   chan - DMA channel
 *   tx   - true: TX mode; false: RX mode
 *   type - block size type
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_dma_set_ext_memblk(int chan, bool tx,
                                enum esp32s3_dma_ext_memblk_e type);

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

/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_dma.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_DMA_H
#define __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_DMA_H

#include <nuttx/config.h>
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

#define ESP32S2_DMA_DATALEN_MAX       (0x1000 - 4)

/* DMA max buffer length */

#define ESP32S2_DMA_BUFLEN_MAX        ESP32S2_DMA_DATALEN_MAX

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DMA descriptor type */

struct esp32s2_dmadesc_s
{
  uint32_t ctrl;                    /* DMA control block */
  uint8_t *pbuf;                    /* DMA TX/RX buffer address */
  struct esp32s2_dmadesc_s *next;   /* Next DMA descriptor address */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s2_dma_init
 *
 * Description:
 *   Initialize DMA outlink descriptors and bind the target buffer to
 *   these DMA descriptors.
 *
 * Input Parameters:
 *   dmadesc - Pointer to the DMA descriptors
 *   num     - Number of DMA descriptors
 *   pbuf    - RX/TX buffer pointer
 *   len     - RX/TX buffer length
 *
 * Returned Value:
 *   Bound pbuf data bytes
 *
 ****************************************************************************/

uint32_t esp32s2_dma_init(struct esp32s2_dmadesc_s *dmadesc, uint32_t num,
                          uint8_t *pbuf, uint32_t len);

/****************************************************************************
 * Name: esp32s2_dma_init_with_padding
 *
 * Description:
 *   Initialize DMA outlink descriptors and bind the target buffer to
 *   these DMA descriptors. If len is not word-aligned, add a new descriptor
 *   containing a 4-byte variable to make the outlink data world-aligned.
 *
 * Input Parameters:
 *   dmadesc - Pointer to the DMA descriptors
 *   num     - Number of DMA descriptors
 *   pbuf    - RX/TX buffer pointer
 *   len     - RX/TX buffer length
 *   stuff   - Value to be padded with the buffer
 *
 * Returned Value:
 *   Bound pbuf data bytes
 *
 ****************************************************************************/

uint32_t esp32s2_dma_init_with_padding(struct esp32s2_dmadesc_s *dmadesc,
                                       uint32_t num,
                                       uint8_t *pbuf,
                                       uint32_t len,
                                       uint32_t *stuff);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_DMA_H */

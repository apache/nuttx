/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_dma.c
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

#include <nuttx/config.h>

#include <assert.h>
#include <sys/param.h>
#include <sys/types.h>

#include "hardware/esp32s2_dma.h"
#include "esp32s2_dma.h"

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

#ifndef ALIGN_UP
#  define ALIGN_UP(num, align) (((num) + ((align) - 1)) & ~((align) - 1))
#endif

/****************************************************************************
 * Public Functions
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
                          uint8_t *pbuf, uint32_t len)
{
  int i;
  uint32_t bytes = len;
  uint8_t *pdata = pbuf;
  uint32_t data_len;
  uint32_t buf_len;

  DEBUGASSERT(dmadesc != NULL);
  DEBUGASSERT(pbuf != NULL);
  DEBUGASSERT(len > 0);

  for (i = 0; i < num; i++)
    {
      data_len = MIN(bytes, ESP32S2_DMA_DATALEN_MAX);

      /* Buffer length must be rounded to next 32-bit boundary. */

      buf_len = ALIGN_UP(data_len, sizeof(uintptr_t));

      dmadesc[i].ctrl = (data_len << DMA_CTRL_DATALEN_S) |
                        (buf_len << DMA_CTRL_BUFLEN_S) |
                        DMA_CTRL_OWN;
      dmadesc[i].pbuf = pdata;
      dmadesc[i].next = &dmadesc[i + 1];

      bytes -= data_len;
      if (bytes == 0)
        {
          break;
        }

      pdata += data_len;
    }

  dmadesc[i].ctrl |= DMA_CTRL_EOF;
  dmadesc[i].next  = NULL;

  return len - bytes;
}

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
                                       uint32_t *stuff)
{
  int i;
  uint32_t bytes = len;
  uint8_t *pdata = pbuf;
  uint32_t data_len = 0;
  uint32_t buf_len = 0;

  DEBUGASSERT(dmadesc != NULL);
  DEBUGASSERT(pbuf != NULL);
  DEBUGASSERT(len > 0);

  for (i = 0; i < num - 1; i++)
    {
      data_len = MIN(bytes, ESP32S2_DMA_DATALEN_MAX);

      /* Buffer length must be rounded to next 32-bit boundary. */

      buf_len = ALIGN_UP(data_len, sizeof(uintptr_t));

      dmadesc[i].ctrl = (data_len << DMA_CTRL_DATALEN_S) |
                        (buf_len << DMA_CTRL_BUFLEN_S) |
                        DMA_CTRL_OWN;
      dmadesc[i].pbuf = pdata;
      dmadesc[i].next = &dmadesc[i + 1];

      bytes -= data_len;
      if (bytes == 0)
        {
          break;
        }

      pdata += data_len;
    }

  /* Check if the data_len of the last descriptor is different from buf_len.
   * If so, it's necessary to add the padding bytes to a new descriptor on
   * outlink.
   */

  if (data_len != buf_len)
    {
      i++;
      dmadesc[i].ctrl = ((buf_len - data_len) << DMA_CTRL_DATALEN_S) |
                        (4 << DMA_CTRL_BUFLEN_S) |
                        DMA_CTRL_OWN;
      dmadesc[i].pbuf = (uint8_t *)stuff;
    }

  dmadesc[i].ctrl |= DMA_CTRL_EOF;
  dmadesc[i].next  = NULL;

  return len - bytes;
}

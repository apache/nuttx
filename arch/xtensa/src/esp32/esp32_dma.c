/****************************************************************************
 * arch/xtensa/src/esp32/esp32_dma.c
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
#include <sys/types.h>

#include "hardware/esp32_dma.h"
#include "esp32_dma.h"

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif

#ifndef ALIGN_UP
#  define ALIGN_UP(num, align) (((num) + ((align) - 1)) & ~((align) - 1))
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_dma_init
 *
 * Description:
 *   Initialize DMA descriptions and bind the target
 *   buffer to these DMA descriptions.
 *
 * Input Parameters:
 *   dmadesc - DMA descriptions pointer
 *   num     - DMA descriptions number
 *   pbuf    - RX/TX buffer pointer
 *   len     - RX/TX buffer length
 *
 * Returned Value:
 *   Binded pbuf data bytes
 *
 ****************************************************************************/

uint32_t esp32_dma_init(struct esp32_dmadesc_s *dmadesc, uint32_t num,
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
      data_len = MIN(bytes, ESP32_DMA_DATALEN_MAX);

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

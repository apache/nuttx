/****************************************************************************
 * arch/xtensa/src/esp32s2/hardware/esp32s2_dma.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_DMA_H
#define __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_DMA_H

/* DMA descriptor */

#define DMA_CTRL_OWN          (1 << 31)   /* Owned by DMA */
#define DMA_CTRL_EOF          (1 << 30)   /* End of frame */
#define DMA_CTRL_SOSF         (1 << 29)   /* Start of sub-frame */
#define DMA_CTRL_OFFSET_S     (24)        /* buffer offset shift */
#define DMA_CTRL_OFFSET_V     (0x1f)      /* buffer offset value */
#define DMA_CTRL_DATALEN_S    (12)        /* received/sent data length shift */
#define DMA_CTRL_DATALEN_V    (0xfff)     /* received/sent data length value */
#define DMA_CTRL_BUFLEN_S     (0)         /* received/sent buffer length shift */
#define DMA_CTRL_BUFLEN_V     (0xfff)     /* received/sent buffer length value */

#endif /* __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_DMA_H */

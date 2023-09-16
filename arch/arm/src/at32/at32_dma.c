/****************************************************************************
 * arch/arm/src/at32/at32_dma.c
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

#include "chip.h"

/* This file is only a thin shell that includes the correct DMA
 * implementation for the selected AT32 IP core:
 *   - AT32 DMA IP version 1 - F43X
 *
 */

#if defined(CONFIG_AT32_HAVE_IP_DMA_V1)
#  if defined(CONFIG_AT32_HAVE_DMAMUX)
#    include "at32_dma_v1mux.c"
#  else
#    include "at32_dma_v1.c"
#  endif
#elif defined(CONFIG_AT32_HAVE_IP_DMA_V2)
#  include "at32_dma_v2.c"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

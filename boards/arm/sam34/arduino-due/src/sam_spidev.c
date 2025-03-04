/****************************************************************************
 * boards/arm/sam34/arduino-due/src/sam_spidev.c
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

#include <debug.h>
#include <arch/board/board.h>

#include "sam_spi.h"
#include "hardware/sam3x_pinmap.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void sam_spi0select(uint32_t devid, bool selected)
{
  spiinfo("devid: %08x CS: %s\n",
          (unsigned int)devid, selected ? "assert" : "de-assert");
  sam_gpiowrite(GPIO_SPI0_CS, !selected);
}

uint8_t sam_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
  return 0;
}

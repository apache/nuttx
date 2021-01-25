/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_irq.c
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
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "cxd56_audio_irq.h"
#include "cxd56_audio_dma.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void cxd56_audio_irq_attach(void)
{
  irq_attach(CXD56_IRQ_AUDIO_0, (xcpt_t)cxd56_audio_dma_int_handler, NULL);
  irq_attach(CXD56_IRQ_AUDIO_1, (xcpt_t)cxd56_audio_dma_int_handler, NULL);
  irq_attach(CXD56_IRQ_AUDIO_2, (xcpt_t)cxd56_audio_dma_int_handler, NULL);
  irq_attach(CXD56_IRQ_AUDIO_3, (xcpt_t)cxd56_audio_dma_int_handler, NULL);
}

void cxd56_audio_irq_detach(void)
{
  irq_detach(CXD56_IRQ_AUDIO_0);
  irq_detach(CXD56_IRQ_AUDIO_1);
  irq_detach(CXD56_IRQ_AUDIO_2);
  irq_detach(CXD56_IRQ_AUDIO_3);
}

void cxd56_audio_irq_enable(void)
{
  up_enable_irq(CXD56_IRQ_AUDIO_0);
  up_enable_irq(CXD56_IRQ_AUDIO_1);
  up_enable_irq(CXD56_IRQ_AUDIO_2);
}

void cxd56_audio_irq_disable(void)
{
  up_disable_irq(CXD56_IRQ_AUDIO_0);
  up_disable_irq(CXD56_IRQ_AUDIO_1);
  up_disable_irq(CXD56_IRQ_AUDIO_2);
}

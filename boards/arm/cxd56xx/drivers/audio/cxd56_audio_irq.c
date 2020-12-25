/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_irq.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

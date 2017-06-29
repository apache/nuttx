/****************************************************************************
 * arch/arm/src/armv7-m/up_itm.c
 *
 *   Copyright (c) 2009 - 2013 ARM LIMITED
 *
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of ARM nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *  *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *   Copyright (C) 2014 Pierre-noel Bouteville . All rights reserved.
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Authors: Pierre-noel Bouteville <pnb990@gmail.com>
 *            Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include <stdint.h>

#include "up_arch.h"
#include "itm.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: itm_sendchar
 *
 * Description:
 *   The function transmits a character via the ITM channel 0, and
 *   - Just returns when no debugger is connected that has booked the output.
 *   - Is blocking when a debugger is connected, but the previous character
 *     sent has not been transmitted.
 *
 * Input Parameters:
 *   ch - Character to transmit.
 *
 * Returned Value:
 *   Character to transmit.
 *
 ****************************************************************************/

uint32_t itm_sendchar(uint32_t ch)
{
  if ((getreg32(ITM_TCR) & ITM_TCR_ITMENA_Msk) && /* ITM enabled */
      (getreg32(ITM_TER) & (1UL << 0)))           /* ITM Port #0 enabled */
    {
      while (getreg32(ITM_PORT(0)) == 0);
      putreg8((uint8_t)ch, ITM_PORT(0));
    }

  return ch;
}

/****************************************************************************
 * Name: itm_receivechar
 *
 * Description:
 *
 * Input Parameters:
 *  The function inputs a character via the external variable g_itm_rxbuffer.
 *
 * Returned Value:
 *   Received character or -1 No character pending.
 *
 ****************************************************************************/

int32_t itm_receivechar(void)
{
  int32_t ch = -1;  /* Assume no character available */

  if (g_itm_rxbuffer != ITM_RXBUFFER_EMPTY)
    {
      ch = g_itm_rxbuffer;
      g_itm_rxbuffer = ITM_RXBUFFER_EMPTY; /* Ready for next character */
    }

  return ch;
}

/****************************************************************************
 * Name: itm_checkchar
 *
 * Description:
 *
 * Input Parameters:
 *  The function checks whether a character is pending for reading in the
 *  variable g_itm_rxbuffer.
 *
 * Returned Value:
 *   0  No character available.
 *   1  Character available.
 *
 ****************************************************************************/

int32_t itm_checkchar (void)
{
  return (g_itm_rxbuffer != ITM_RXBUFFER_EMPTY);
}

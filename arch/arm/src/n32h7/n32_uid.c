/****************************************************************************
 * arch/arm/src/n32h7/n32_uid.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2015 Marawan Ragab. All rights reserved.
 * SPDX-FileContributor: Marawan Ragab <marawan31@gmail.com>
 * SPDX-FileContributor: David Sidrane <david.sirane@nscdg.com>
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

#include "hardware/n32h7_memorymap.h"

#include "n32_uid.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void n32_get_uniqueid(uint8_t uniqueid[16])
{
  uint32_t otpc_tmp;

  for (int i = 0; i < 4; i++)
    {
      putreg32(N32_SYSMEM_UID + i, N32H7_OTPC_ADDR);
      usleep(25000);
      otpc_tmp = getreg32(N32H7_OTPC_DATA);
      uniqueid[i * 4 + 0] = (otpc_tmp >> 0) & 0xff;
      uniqueid[i * 4 + 1] = (otpc_tmp >> 8) & 0xff;
      uniqueid[i * 4 + 2] = (otpc_tmp >> 16) & 0xff;
      uniqueid[i * 4 + 3] = (otpc_tmp >> 24) & 0xff;
    }
}

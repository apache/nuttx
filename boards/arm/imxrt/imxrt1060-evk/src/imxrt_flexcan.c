/****************************************************************************
 * boards/arm/imxrt/imxrt1060-evk/src/imxrt_flexcan.c
 *
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

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/can/can.h>

#include "imxrt_flexcan.h"
#include "imxrt1060-evk.h"

#ifdef CONFIG_IMXRT_FLEXCAN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

int imxrt_can_setup(void)
{
  int ret;
#if defined(CONFIG_IMXRT_FLEXCAN3) && defined(CONFIG_IMXRT_FLEXCAN2)
  canerr("ERROR: Only one FlexCAN interface can be defined at the same time on imxrt1060-evk\n");
  return -ENODEV;
#endif

#ifdef CONFIG_IMXRT_FLEXCAN3
  /* Call arm_caninitialize() to get an instance of the CAN interface */

  ret = imxrt_caninitialize(3);
  if (ret < 0)
    {
      canerr("ERROR: Failed to get CAN interface\n");
      return -ENODEV;
    }
#elif CONFIG_IMXRT_FLEXCAN2
  ret = imxrt_caninitialize(2);
  if (ret < 0)
    {
      canerr("ERROR: Failed to get CAN interface\n");
      return -ENODEV;
    }
#elif CONFIG_IMXRT_FLEXCAN1
  canerr("ERROR: FlexCAN1 is not available on imxrt1060-evk. Please choose FlexCAN2 or FlexCAN3 (CAN_FD available)\n");
  return -ENODEV;
#else
  return -ENODEV;
#endif
  return OK;
}

#endif /* CONFIG_IMXRT_FLEXCAN */

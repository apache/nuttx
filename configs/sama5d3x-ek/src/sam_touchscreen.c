/************************************************************************************
 * configs/sama5d3x-ek/src/sam_touchscreen.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include "sam_adc.h"
#include "sam_tsd.h"
#include "sama5d3x-ek.h"

#ifdef CONFIG_SAMA5_TSD

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_INPUT
#  error "Touchscreen support requires CONFIG_INPUT"
#endif

#ifndef CONFIG_SAMA5D3xEK_TSD_DEVMINOR
#  define CONFIG_SAMA5D3xEK_TSD_DEVMINOR 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arch_tcinitialize
 *
 * Description:
 *   Each board that supports a touchscreen device must provide this
 *   function.  This function is called by application-specific, setup logic
 *   to configure the touchscreen device.  This function will register the
 *   driver as /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int arch_tcinitialize(int minor)
{
  struct sam_adc_s *adc;
  static bool initialized = false;
  int ret;

  idbg("initialized:%d minor:%d\n", initialized, minor);
  DEBUGASSERT(minor == 0);

  /* Since there is no uninitialized logic, this initialization can be
   * performed only one time.
   */

  if (!initialized)
    {
      /* Initialize the ADC driver */

      adc = sam_adc_initialize();
      if (!adc)
        {
          idbg("ERROR: Failed to initialize the ADC driver\n");
          return -ENODEV;
        }

      /* Initialize and register the SPI touchscreen device */

      ret = sam_tsd_register(adc, CONFIG_SAMA5D3xEK_TSD_DEVMINOR);
      if (ret < 0)
        {
          idbg("ERROR: Failed to register touchscreen device /dev/input%d: %d\n",
               CONFIG_SAMA5D3xEK_TSD_DEVMINOR, ret);
          return -ENODEV;
        }

      initialized = true;
    }

  return OK;
}

/****************************************************************************
 * Name: arch_tcuninitialize
 *
 * Description:
 *   Each board that supports a touchscreen device must provide this function.
 *   This function is called by application-specific, setup logic to
 *   uninitialize the touchscreen device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void arch_tcuninitialize(void)
{
  /* No support for un-initializing the touchscreen  yet */
}

#endif /* CONFIG_INPUT_ADS7843E */

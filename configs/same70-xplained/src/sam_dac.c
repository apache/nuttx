/************************************************************************************
 * configs/same70-xplained/src/sam_dac.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "sam_dac.h"
#include "same70-xplained.h"

#if defined(CONFIG_SAMV7_DAC0) || defined(CONFIG_SAMV7_DAC1)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_dacdev_initialize
 *
 * Description:
 *   Called to configure DAC peripheral module and register DAC device driver
 ************************************************************************************/

int sam_dacdev_initialize(void)
{
    static bool initialized = false;
    struct dac_dev_s *dac;
    int ret;

    /* Check if we have already initialized */

    if (!initialized)
      {
#ifdef CONFIG_SAMV7_DAC0
        /* Get an instance of the DAC0 interface */

        dac = sam_dac_initialize(0);
        if (dac == NULL)
          {
            aerr("ERROR:  Failed to get DAC0 interface\n");
            return -ENODEV;
          }

        /* Register the DAC driver at "/dev/dac0" */

        ret = dac_register("/dev/dac0", dac);
        if (ret < 0)
          {
            aerr("ERROR: dac_register failed: %d\n", ret);
            return ret;
          }
#endif

#ifdef CONFIG_SAMV7_DAC1
        /* Get an instance of the DAC1 interface */

        dac = sam_dac_initialize(1);
        if (dac == NULL)
          {
            aerr("ERROR:  Failed to get DAC1 interface\n");
            return -ENODEV;
          }

        /* Register the DAC driver at "/dev/dac1" */

        ret = dac_register("/dev/dac1", dac);
        if (ret < 0)
          {
            aerr("ERROR: dac_register failed: %d\n", ret);
            return ret;
          }
#endif
        /* Now we are initialized */

        initialized = true;
      }
    return OK;
}

#endif /* CONFIG_SAMV7_DAC0 || CONFIG_SAMV7_DAC1 */

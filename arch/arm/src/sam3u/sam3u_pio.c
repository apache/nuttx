/****************************************************************************
 * arch/arm/src/sam3u/sam3u_pio.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "sam3u_internal.h"
#include "sam3u_pio.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/************************************************************************************
 * Name: sam3u_configinput
 *
 * Description:
 *   Configure a GPIO input pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

static inline int sam3u_configinput(uint16_t cfgset)
{
#warning "Not implemented"
return -ENOSYS;
}

/************************************************************************************
 * Name: sam3u_configoutput
 *
 * Description:
 *   Configure a GPIO output pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

static inline int sam3u_configoutput(uint16_t cfgset)
{
#warning "Not implemented"
return -ENOSYS;
}

/************************************************************************************
 * Name: sam3u_configperipha
 *
 * Description:
 *   Configure a GPIO pin driven by a peripheral A signal based on bit-encoded
 *   description of the pin.
 *
 ************************************************************************************/

static inline int sam3u_configperipha(uint16_t cfgset)
{
#warning "Not implemented"
return -ENOSYS;
}

/************************************************************************************
 * Name: sam3u_configperipha
 *
 * Description:
 *   Configure a GPIO pin driven by a peripheral A signal based on bit-encoded
 *   description of the pin.
 *
 ************************************************************************************/

static inline int sam3u_configperiphb(uint16_t cfgset)
{
#warning "Not implemented"
return -ENOSYS;
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/************************************************************************************
 * Name: sam3u_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

int sam3u_configgpio(uint16_t cfgset)
{
  int ret;

  switch (cfgset & GPIO_MODE_MASK)
    {    
      case GPIO_INPUT:
        ret = sam3u_configinput(cfgset);
        break;
    
      case GPIO_OUTPUT:
        ret = sam3u_configoutput(cfgset);
        break;
    
      case GPIO_PERIPHA:
        ret = sam3u_configperipha(cfgset);
        break;
   
      case GPIO_PERIPHB:
        ret = sam3u_configperiphb(cfgset);
        break;
    
      default:
        ret = -EINVAL;
        break;
    }
  return ret;
}

/************************************************************************************
 * Name: sam3u_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void sam3u_gpiowrite(uint16_t pinset, bool value)
{
#warning "Not implemented"
}

/************************************************************************************
 * Name: sam3u_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool sam3u_gpioread(uint16_t pinset)
{
#warning "Not implemented"
return false;
}

/************************************************************************************
 * Function:  sam3u_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the port of the provided pin description.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG
int sam3u_dumpgpio(uint16_t pinset, const char *msg)
{
#warning "Not implemented"
return -ENOSYS;
}
#endif

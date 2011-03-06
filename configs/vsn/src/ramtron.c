/****************************************************************************
 * config/vsn/src/ramtron.c
 * arch/arm/src/board/ramtron.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *
 *   Authors: Uros Platise <uros.platise@isotel.eu>
 *            Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#ifdef CONFIG_STM32_SPI3
#  include <nuttx/spi.h>
#  include <nuttx/mtd.h>
#endif

#include "vsn.h"



int up_ramtron(void)
{
#ifdef CONFIG_STM32_SPI3
  FAR struct spi_dev_s *spi;
  FAR struct mtd_dev_s *mtd;
  int retval;

  /* Get the SPI port */

  message("nsh_archinitialize: Initializing SPI port 3\n");
  spi = up_spiinitialize(3);
  if (!spi)
    {
      message("nsh_archinitialize: Failed to initialize SPI port 3\n");
      return -ENODEV;
    }
  message("nsh_archinitialize: Successfully initialized SPI port 3\n");

  message("nsh_archinitialize: Bind SPI to the SPI flash driver\n");
  mtd = ramtron_initialize(spi);
  if (!mtd)
    {
      message("nsh_archinitialize: Failed to bind SPI port 0 to the SPI FLASH driver\n");
      return -ENODEV;
    }
  message("nsh_archinitialize: Successfully bound SPI port 0 to the SPI FLASH driver\n");

  retval = ftl_initialize(0,NULL, mtd);
  message("FTL returned with %d\n", retval);
#endif
  return OK;
}

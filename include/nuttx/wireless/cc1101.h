/****************************************************************************
 * include/nuttx/wireless/cc1101.h
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *
 *   Authors: Uros Platise <uros.platise@isotel.eu>
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

/** \file
 *  \author Uros Platise
 *  \brief Chipcon CC1101 Device Driver
 **/ 

#ifndef __INCLUDE_NUTTX_WIRELESS_CC1101_H
#define __INCLUDE_NUTTX_WIRELESS_CC1101_H

#include <nuttx/config.h>
#include <nuttx/spi.h>

#include <stdint.h>
#include <stdbool.h>

/************************************************************************************
 * Pre-Processor Declarations
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif


/************************************************************************************
 * Public Data Types
 ************************************************************************************/

struct cc1101_dev_s;


/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/** Initialize Chipcon CC1101 Chip
 * 
 * \param spi SPI Device Structure
 * \return Pointer to newly allocated CC1101 structure or NULL on error with errno set.
 * 
 * Possible errno as set by this function on error:
 *  - ENODEV: When device addressed is not compatible or it is not a CC1101
 *  - EFAULT: When there is no device
 *  - EBUSY: When device is already addressed by other device driver (not yet supported by low-level driver)
 **/
EXTERN struct cc1101_dev_s * cc1101_init(struct spi_dev_s * spi);

/** Deinitialize Chipcon CC1101 Chip
 * 
 * \param dev Device to CC1101 device structure, as returned by the cc1101_init()
 * \return OK On success
 * 
 **/
EXTERN int cc1101_deinit(struct cc1101_dev_s * dev);

/** Power up device, start conversion */
EXTERN int cc1101_powerup(struct cc1101_dev_s * dev);

/** Power down device, stop conversion */
EXTERN int cc1101_powerdown(struct cc1101_dev_s * dev);

/** Set Multi Purpose Output Function */
EXTERN int cc1101_setgdo(struct cc1101_dev_s * dev, uint8_t pin, uint8_t function);


#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_WIRELESS_CC1101_H */

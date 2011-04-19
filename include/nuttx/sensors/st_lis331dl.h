/****************************************************************************
 * include/nuttx/sensors/st_lis331dl.h
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
 *  \brief ST LIS331DL I2C Device Driver
 **/ 

#ifndef __INCLUDE_NUTTX_SENSORS_ST_LIS331DL_H
#define __INCLUDE_NUTTX_SENSORS_ST_LIS331DL_H

#include <nuttx/i2c.h>
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
 * LIS331DL Internal Registers
 ************************************************************************************/

#define ST_LIS331DL_WHOAMI          0x0F    /* who am I register */
#define ST_LIS331DL_WHOAMI_VALUE    0x3B    /* Valid result is 0x3B */

#define ST_LIS331DL_CTRL_REG1       0x20
#define ST_LIS331DL_CR1_DR          0x80    /* Data-rate selection 0: 100 Hz, 1: 400 Hz */
#define ST_LIS331DL_CR1_PD          0x40    /* Active Mode (1) / Power-down (0) */
#define ST_LIS331DL_CR1_FS          0x20    /* Full Scale (1) +-9g or Normal Scale (0) +-2g */
#define ST_LIS331DL_CR1_ST          0x18    /* Self test enable */
#define ST_LIS331DL_CR1_ZEN         0x04    /* Z-Axis Enable */
#define ST_LIS331DL_CR1_YEN         0x02    /* Y-Axis Enable */
#define ST_LIS331DL_CR1_XEN         0x01    /* X-Axis Enable */

#define ST_LIS331DL_CTRL_REG2       0x21
#define ST_LIS331DL_CTRL_REG3       0x22

#define ST_LIS331DL_HP_FILTER_RESET 0x23

#define ST_LIS331DL_STATUS_REG      0x27    /* Status Register */
#define ST_LIS331DL_SR_ZYXOR        0x80    /* OR'ed X,Y and Z data over-run  */
#define ST_LIS331DL_SR_ZOR          0x40    /* individual data over-run ... */
#define ST_LIS331DL_SR_YOR          0x20
#define ST_LIS331DL_SR_XOR          0x10
#define ST_LIS331DL_SR_ZYXDA        0x08    /* OR'ed X,Y and Z data available */
#define ST_LIS331DL_SR_ZDA          0x04    /* individual data available ... */
#define ST_LIS331DL_SR_YDA          0x02
#define ST_LIS331DL_SR_XDA          0x01

#define ST_LIS331DL_OUT_X           0x29
#define ST_LIS331DL_OUT_Y           0x2B
#define ST_LIS331DL_OUT_Z           0x2D


/************************************************************************************
 * Public Data Types
 ************************************************************************************/

struct st_lis331dl_dev_s;

struct st_lis331dl_vector_s {
    int8_t  x, y, z;
};


/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/** Initialize ST LIS331DL Chip
 * 
 * \param i2c I2C Device Structure
 * \param address I2C Address of the proposed device
 * \return Pointer to newly allocated ST LIS331DL structure or NULL on error with errno set.
 * 
 * Possible errno as set by this function on error:
 *  - ENODEV: When device addressed on given address is not compatible or it is not a LIS331DL
 *  - EFAULT: When there is no device at given address.
 *  - EBUSY: When device is already addressed by other device driver (not yet supported by low-level driver)
 **/
EXTERN struct st_lis331dl_dev_s * st_lis331dl_init(struct i2c_dev_s * i2c, uint16_t address);

/** Deinitialize ST LIS331DL Chip
 * 
 * \param dev Device to LIS331DL device structure, as returned by the st_lis331dl_init()
 * \return OK On success
 * 
 **/
EXTERN int st_lis331dl_deinit(struct st_lis331dl_dev_s * dev);

/** Power up device, start conversion */
EXTERN int st_lis331dl_powerup(struct st_lis331dl_dev_s * dev);

/** Power down device, stop conversion */
EXTERN int st_lis331dl_powerdown(struct st_lis331dl_dev_s * dev);

/** Configure conversion 
 * 
 * \param dev Device to LIS331DL device structure
 * \param full When set, range of [-9g, 9g] is selected, otherwise [-2g, +2g]
 * \param fast When set, conversion operates at 400 Hz, otherwise at 100 Hz
 * \return OK on success or errno is set
 **/
EXTERN int st_lis331dl_setconversion(struct st_lis331dl_dev_s * dev, bool full, bool fast);

/** Get precision
 * 
 * \return Precision of 1 LSB in terms of unit [mg]
 **/
EXTERN int st_lis331dl_getprecision(struct st_lis331dl_dev_s * dev);

/** Get sample rate 
 * 
 * \return Sample rate in unit of [Hz]
 **/
EXTERN int st_lis331dl_getsamplerate(struct st_lis331dl_dev_s * dev);

/** Get readings, updates internal data structure
 * 
 * \param dev Device to LIS331DL device structure
 * \return Ptr to vector acceleration [x,y,z] on success, or NULL on error with errno set.
 *   If data is not yet ready to be read from the LIS331 then errno is set to EAGAIN otherwise
 *   errno is set by I2C_TRANSFER().
 */
EXTERN const struct st_lis331dl_vector_s * st_lis331dl_getreadings(struct st_lis331dl_dev_s * dev);



#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_SENSORS_ST_LIS331DL_H */


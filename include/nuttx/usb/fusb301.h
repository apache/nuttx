/****************************************************************************
 * include/nuttx/usb/fusb301.h
 * FUSB301 USB type-C controller driver
 *
 *   Copyright (C) 2016-2017 Haltian Ltd. All rights reserved.
 *   Authors: Harri Luhtala <harri.luhtala@haltian.com>
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

#ifndef __INCLUDE_NUTTX_USB_FUSB301_H
#define __INCLUDE_NUTTX_USB_FUSB301_H

#include <nuttx/fs/ioctl.h>

/************************************************************************************
 * Pre-Processor Declarations
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* IOCTL Commands ***********************************************************/

#define USBCIOC_READ_DEVID    _USBCIOC(0x0001)  /* Arg: uint8_t* pointer */
#define USBCIOC_SETUP         _USBCIOC(0x0002)  /* Arg: uint8_t* pointer */
#define USBCIOC_SET_MODE      _USBCIOC(0x0003)  /* Arg: uint8_t value */
#define USBCIOC_SET_STATE     _USBCIOC(0x0004)  /* Arg: uint8_t value */
#define USBCIOC_READ_STATUS   _USBCIOC(0x0005)  /* Arg: uint8_t* pointer*/
#define USBCIOC_READ_DEVTYPE  _USBCIOC(0x0006)  /* Arg: uint8_t* pointer*/
#define USBCIOC_RESET         _USBCIOC(0x0007)  /* Arg: None */

/************************************************************************************
 * Public Types
 ************************************************************************************/

enum fusb301_reg_address_e
{
  FUSB301_DEV_ID_REG = 0x01,
  FUSB301_MODE_REG,
  FUSB301_CONTROL_REG,
  FUSB301_MANUAL_REG,
  FUSB301_RESET_REG,
  FUSB301_MASK_REG = 0x10,
  FUSB301_STATUS_REG,
  FUSB301_TYPE_REG,
  FUSB301_INTERRUPT_REG
};

/* Device ID - 0x01 */

enum fusb301_devid_mask_e
{
  DEV_ID_REVISION_MASK = 0x0F,
  DEV_ID_VERSION_MASK  = 0xF0
};

#define DEV_ID_VER_A  0x10
#define DEV_ID_REV_C  0x02

/* Modes - 0x02 */

enum fusb301_mode_e
{
  MODE_SRC     = (1 << 0),
  MODE_SRC_ACC = (1 << 1),
  MODE_SNK     = (1 << 2),
  MODE_SNK_ACC = (1 << 3),
  MODE_DRP     = (1 << 4),
  MODE_DRP_ACC = (1 << 5)
};

/* Control - 0x03 */

enum fusb301_control_e
{
  CONTROL_INT_ENABLE   = (0 << 0),
  CONTROL_INT_DISABLE  = (1 << 0),
  CONTROL_CUR_DISABLED = (0 << 1),
  CONTROL_CUR_DEFAULT  = (1 << 1),
  CONTROL_CUR_1500     = (2 << 1),
  CONTROL_CUR_3000     = (3 << 1),
  CONTROL_TGL_35_15MS  = (0 << 4),
  CONTROL_TGL_30_20MS  = (1 << 4),
  CONTROL_TGL_25_25MS  = (2 << 4),
  CONTROL_TGL_20_30MS  = (3 << 4)
};

/* Manual - 0x04 */

enum fusb301_manual_e
{
  MANUAL_ERROR_REC = (1 << 0),
  MANUAL_DISABLED  = (1 << 1),
  MANUAL_UNATT_SRC = (1 << 2),
  MANUAL_UNATT_SNK = (1 << 3)
};

/* Reset - 0x05 */

enum fusb301_reset_e
{
  RESET_SW_RES = (1 << 0)
};

/* Interrupt mask - 0x10 */

enum fusb301_int_mask_e
{
  INT_MASK_ATTACK = (1 << 0),
  INT_MASK_DETACH = (1 << 1),
  INT_MASK_BC_LVL = (1 << 2),
  INT_MASK_ACC_CH = (1 << 3)
};

/* Status - 0x11 */

enum fusb301_status_e
{
  STATUS_ATTACH        = (1 << 0),
  STATUS_BC_SINK_UNATT = (0 << 1),
  STATUS_BC_SINK_DEF   = (1 << 1),
  STATUS_BC_SINK_1500  = (2 << 1),
  STATUS_BC_SINK_3000  = (3 << 1),
  STATUS_VBUS_OK       = (1 << 3),
  STATUS_CC_NO_CONN    = (0 << 4),
  STATUS_CC_1          = (1 << 4),
  STATUS_CC_2          = (2 << 4),
  STATUS_CC_FAULT      = (3 << 4)
};

/* Type - 0x12 */

enum fusb301_type_e
{
  TYPE_AUDIOACC = (1 << 0),
  TYPE_DEBUGACC = (1 << 1),
  TYPE_SOURCE   = (1 << 3),
  TYPE_SINK     = (1 << 4)
};

/* Interrupt - 0x13 */

enum fusb301_interrupt_e
{
  INTERRUPT_ATTACH = (1 << 0),
  INTERRUPT_DETACH = (1 << 1),
  INTERRUPT_BC_LVL = (1 << 2),
  INTERRUPT_ACC_CH = (1 << 3)
};

struct fusb301_result_s
{
  uint8_t status;
  uint8_t dev_type;
};

struct fusb301_setup_s
{
  uint8_t drp_toggle_timing;
  uint8_t host_current;
  uint8_t int_mask;
  bool global_int_mask;
};

struct fusb301_config_s
{
  /* Device characterization */

  int irq;

  CODE int  (*irq_attach)(FAR struct fusb301_config_s *state, xcpt_t isr,
                          FAR void *arg);
  CODE void (*irq_enable)(FAR struct fusb301_config_s *state, bool enable);
  CODE void (*irq_clear)(FAR struct fusb301_config_s *state);
};

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/****************************************************************************
 * Name: fusb301_register
 *
 * Description:
 *   Register the FUSB301 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/usbc0"
 *   i2c     - An instance of the I2C interface to use to communicate with FUSB301
 *   addr    - The I2C address of the FUSB301. The I2C address of the FUSB301 is 0x25.
 *   config  - Pointer to FUSB301 configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int fusb301_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr, FAR struct fusb301_config_s *config);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_FUSB301_H */

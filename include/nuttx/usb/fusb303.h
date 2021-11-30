/****************************************************************************
 * include/nuttx/usb/fusb303.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_USB_FUSB303_H
#define __INCLUDE_NUTTX_USB_FUSB303_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

#define USBCIOC_READ_DEVID    _USBCIOC(0x0001)  /* Arg: uint8_t* pointer */
#define USBCIOC_SETUP         _USBCIOC(0x0002)  /* Arg: uint8_t* pointer */
#define USBCIOC_SET_MODE      _USBCIOC(0x0003)  /* Arg: uint8_t value */
#define USBCIOC_SET_STATE     _USBCIOC(0x0004)  /* Arg: uint8_t value */
#define USBCIOC_READ_STATUS   _USBCIOC(0x0005)  /* Arg: uint8_t* pointer*/
#define USBCIOC_READ_DEVTYPE  _USBCIOC(0x0006)  /* Arg: uint8_t* pointer*/
#define USBCIOC_RESET         _USBCIOC(0x0007)  /* Arg: None */

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum fusb303_reg_address_e
{
  FUSB303_DEV_ID_REG = 0x01,
  FUSB303_DEV_TYPE_REG,
  FUSB303_PORTROLE_REG,
  FUSB303_CONTROL_REG,
  FUSB303_CONTROL1_REG,
  FUSB303_MANUAL_REG = 0x09,
  FUSB303_RESET_REG,
  FUSB303_MASK_REG = 0x0e,
  FUSB303_MASK1_REG,
  FUSB303_STATUS_REG = 0x11,
  FUSB303_STATUS1_REG,
  FUSB303_TYPE_REG,
  FUSB303_INTERRUPT_REG,
  FUSB303_INTERRUPT1_REG
};

/* Device ID - 0x01 */

enum fusb303_devid_mask_e
{
  DEV_ID_REVISION_MASK = 0x0f,
  DEV_ID_VERSION_MASK  = 0xf0
};

#define DEV_ID_VER_A  0x10
#define DEV_ID_REV_A  0x00

/* Device Type - 0x02 */

#define DEV_TYPE_FUSB303  0x01
#define DEV_TYPE_FUSB303T 0x02

/* Port Roles - 0x03 (called Mode here for fusb301 compatibility) */

enum fusb303_mode_e
{
  MODE_SRC       = (1 << 0),
  MODE_SNK       = (1 << 1),
  MODE_DRP       = (1 << 2),
  MODE_AUDIOACC  = (1 << 3),
  MODE_TRY_SNK   = (1 << 4), /* For DRP only */
  MODE_TRY_SRC   = (1 << 5), /* For DRP only */
  MODE_ORIENTDEB = (1 << 6),

  /* Bit 7 reserved */
};

/* Control - 0x04 */

enum fusb303_control_e
{
  CONTROL_INT_ENABLE      = (0 << 0),
  CONTROL_INT_DISABLE     = (1 << 0),
  CONTROL_CUR_RESERVED    = (0 << 1), /* Do not use */
  CONTROL_CUR_DEFAULT     = (1 << 1), /* Default USB Power */
  CONTROL_CUR_1500        = (2 << 1), /* Medium Current Mode */
  CONTROL_CUR_3000        = (3 << 1), /* High Current Mode */
  CONTROL_DCABLE_EN       = (1 << 3),
  CONTROL_DRPTOGGLE_60_40 = (0 << 4), /* 60% in Unattached.SNK and 40% in Unattached.SRC */
  CONTROL_DRPTOGGLE_50_50 = (1 << 4), /* 50% in Unattached.SNK and 50% in Unattached.SRC */
  CONTROL_DRPTOGGLE_40_60 = (2 << 4), /* 40% in Unattached.SNK and 60% in Unattached.SRC */
  CONTROL_DRPTOGGLE_30_70 = (3 << 4), /* 30% in Unattached.SNK and 70% in Unattached.SRC */
  CONTROL_T_DRP_60MS      = (0 << 6), /* Total period of the DRP toggle cycle 60 ms */
  CONTROL_T_DRP_70MS      = (1 << 6), /* Total period of the DRP toggle cycle 70 ms */
  CONTROL_T_DRP_80MS      = (2 << 6), /* Total period of the DRP toggle cycle 80 ms */
  CONTROL_T_DRP_90MS      = (3 << 6), /* Total period of the DRP toggle cycle 90 ms */
};

/* Control 1 - 0x05 */

enum fusb303_control1_e
{
  CONTROL1_TCCDEB_120     = (0 << 0), /* Debounce time 120 ms */
  CONTROL1_TCCDEB_130     = (1 << 0),
  CONTROL1_TCCDEB_140     = (2 << 0),
  CONTROL1_TCCDEB_150     = (3 << 0),
  CONTROL1_TCCDEB_160     = (4 << 0),
  CONTROL1_TCCDEB_170     = (5 << 0),
  CONTROL1_TCCDEB_180     = (6 << 0),
  CONTROL1_ENABLE         = (1 << 3),
  CONTROL1_AUTO_SNK_EN    = (1 << 4),
  CONTROL1_AUTO_SNK_TH_30 = (0 << 5), /* Weak battery VDD threshold voltage 3.0 V */
  CONTROL1_AUTO_SNK_TH_31 = (1 << 5),
  CONTROL1_AUTO_SNK_TH_32 = (2 << 5),
  CONTROL1_AUTO_SNK_TH_33 = (3 << 5),
  CONTROL1_REMEDY_EN      = (1 << 7),
};

/* Manual - 0x09 */

enum fusb303_manual_e
{
  MANUAL_ERROR_REC = (1 << 0),
  MANUAL_DISABLED  = (1 << 1),
  MANUAL_UNATT_SRC = (1 << 2),
  MANUAL_UNATT_SNK = (1 << 3),
  MANUAL_FORCE_SNK = (1 << 4),
  MANUAL_FORCE_SRC = (1 << 5),

  /* Bits 6:7 reserved */
};

/* Reset - 0x0a */

enum fusb303_reset_e
{
  RESET_SW_RES = (1 << 0)
};

/* Interrupt mask - 0x0e */

enum fusb303_int_mask_e
{
  INT_MASK_ATTACH   = (1 << 0),
  INT_MASK_DETACH   = (1 << 1),
  INT_MASK_BC_LVL   = (1 << 2),
  INT_MASK_AUTOSNK  = (1 << 3),
  INT_MASK_VBUS_CHG = (1 << 4),
  INT_MASK_FAULT    = (1 << 5),
  INT_MASK_ORIENT   = (1 << 6),

  /* Bit 7 reserved */
};

/* Interrupt mask1 - 0x0f */

enum fusb303_int_mask1_e
{
  INT_MASK1_REMEDY    = (1 << 0),
  INT_MASK1_FRC_SUCC  = (1 << 1),
  INT_MASK1_FRC_FAIL  = (1 << 2),

  /* Bit 4 reserved */

  INT_MASK1_REM_FAIL  = (1 << 3),
  INT_MASK1_REM_VBON  = (1 << 5),
  INT_MASK1_REM_VBOFF = (1 << 6),

  /* Bit 7 reserved */
};

/* Status - 0x11 */

enum fusb303_status_e
{
  STATUS_ATTACH        = (1 << 0),
  STATUS_BC_LVL_UNATT  = (0 << 1),
  STATUS_BC_LVL_DEF    = (1 << 1),
  STATUS_BC_LVL_1500   = (2 << 1),
  STATUS_BC_LVL_3000   = (3 << 1),
  STATUS_VBUS_OK       = (1 << 3),
  STATUS_CC_NO_CONN    = (0 << 4),
  STATUS_CC_1          = (1 << 4),
  STATUS_CC_2          = (2 << 4),
  STATUS_CC_FAULT      = (3 << 4),
  STATUS_VSAFE0V       = (1 << 6),
  STATUS_AUTOSNK       = (1 << 7),
};

/* Status1 - 0x12 */

enum fusb303_status1_e
{
  STATUS1_REMEDY = (1 << 0),
  STATUS1_FAULT  = (1 << 1),
};

/* Type - 0x13 */

enum fusb303_type_e
{
  TYPE_AUDIO       = (1 << 0),
  TYPE_AUDIOVBUS   = (1 << 1),
  TYPE_ACTIVECABLE = (1 << 2),
  TYPE_SOURCE      = (1 << 3),
  TYPE_SINK        = (1 << 4),
  TYPE_DEBUGSNK    = (1 << 5),
  TYPE_DEBUGSRC    = (1 << 6),

  /* Bit 7 reserved */
};

/* Interrupt - 0x14 */

enum fusb303_interrupt_e
{
  INTERRUPT_ATTACH   = (1 << 0),
  INTERRUPT_DETACH   = (1 << 1),
  INTERRUPT_BC_LVL   = (1 << 2),
  INTERRUPT_AUTOSNK  = (1 << 3),
  INTERRUPT_VBUS_CHG = (1 << 4),
  INTERRUPT_FAULT    = (1 << 5),
  INTERRUPT_ORIENT   = (1 << 6),

  /* Bit 7 reserved */
};

/* Interrupt1 - 0x15 */

enum fusb303_interrupt1_e
{
  INTERRUPT1_REMEDY    = (1 << 0),
  INTERRUPT1_FRC_SUCC  = (1 << 1),
  INTERRUPT1_FRC_FAIL  = (1 << 2),
  INTERRUPT1_REM_FAIL  = (1 << 3),

  /* Bit 4 reserved */

  INTERRUPT1_REM_VBON  = (1 << 5),
  INTERRUPT1_REM_VBOFF = (1 << 6),

  /* Bit 7 reserved */
};

struct fusb303_result_s
{
  uint8_t status;
  uint8_t status1;
  uint8_t dev_type;
};

struct fusb303_setup_s
{
  uint8_t drp_toggle_timing;
  uint8_t host_current;
  bool    dcable_en;
  bool    remedy_en;
  bool    auto_snk_en;
  uint8_t global_int_mask;
  uint8_t int_mask;
  uint8_t int_mask1;
};

struct fusb303_config_s
{
  /* Device characterization */

  int irq;

  CODE int  (*irq_attach)(FAR struct fusb303_config_s *state, xcpt_t isr,
                          FAR void *arg);
  CODE void (*irq_enable)(FAR struct fusb303_config_s *state, bool enable);
  CODE void (*irq_clear)(FAR struct fusb303_config_s *state);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: fusb303_register
 *
 * Description:
 *   Register the FUSB303 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/usbc0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             FUSB303
 *   addr    - The I2C address of the FUSB303. The I2C address of the FUSB303
 *             is either 0x42 or 0x62.
 *   config  - Pointer to FUSB303 configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int fusb303_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr, FAR struct fusb303_config_s *config);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_FUSB303_H */

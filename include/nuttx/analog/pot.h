/****************************************************************************
 * include/nuttx/analog/pot.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_ANALOG_POT_H
#define __INCLUDE_NUTTX_ANALOG_POT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/mutex.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

/* Common IOCTL commands for all digital potentiometer drivers. Optional
 * commands return -ENOTSUP if not implemented by the lower half; support
 * is advertised in pot_info_s caps.
 *
 * Cmd: POTIOC_GET_INFO      Arg: struct pot_info_s *info
 * Cmd: POTIOC_SET_WIPER     Arg: struct pot_wiper_s *wiper
 * Cmd: POTIOC_GET_WIPER     Arg: struct pot_wiper_s *wiper
 * Cmd: POTIOC_MOVE_WIPER    Arg: struct pot_move_s *move
 * Cmd: POTIOC_SET_ENABLE    Arg: struct pot_enable_s *enable
 * Cmd: POTIOC_STORE_WIPER   Arg: struct pot_nv_s *nv
 * Cmd: POTIOC_RECALL_WIPER  Arg: struct pot_nv_s *nv
 */

#define POTIOC_GET_INFO     _ANIOC(AN_POT_FIRST + 0)
#define POTIOC_SET_WIPER    _ANIOC(AN_POT_FIRST + 1)
#define POTIOC_GET_WIPER    _ANIOC(AN_POT_FIRST + 2)
#define POTIOC_MOVE_WIPER   _ANIOC(AN_POT_FIRST + 3)
#define POTIOC_SET_ENABLE   _ANIOC(AN_POT_FIRST + 4)
#define POTIOC_STORE_WIPER  _ANIOC(AN_POT_FIRST + 5)
#define POTIOC_RECALL_WIPER _ANIOC(AN_POT_FIRST + 6)

/* Device capabilities reported in pot_info_s */

#define POT_CAP_READBACK    (1 << 0)  /* Wiper position can be read back */
#define POT_CAP_MOVE        (1 << 1)  /* Relative wiper move supported */
#define POT_CAP_ENABLE      (1 << 2)  /* Wiper enable/shutdown supported */
#define POT_CAP_NV          (1 << 3)  /* Non-volatile store/recall supported */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Device properties for POTIOC_GET_INFO */

struct pot_info_s
{
  uint8_t  nwipers;              /* Number of wipers */
  uint32_t max;                  /* Wiper full scale position */
  uint32_t rab;                  /* Terminal A-B resistance in ohms,
                                  * 0 if unknown. ohms = val * rab / max */
  uint32_t caps;                 /* Capabilities, see POT_CAP_* */
};

/* Wiper access for POTIOC_SET_WIPER and POTIOC_GET_WIPER */

struct pot_wiper_s
{
  uint8_t  wiper;                /* Wiper index */
  uint32_t val;                  /* Wiper value */
};

/* Relative wiper move for POTIOC_MOVE_WIPER */

struct pot_move_s
{
  uint8_t wiper;                 /* Wiper index */
  int32_t steps;                 /* Steps to move, negative moves down */
};

/* Wiper enable control for POTIOC_SET_ENABLE */

struct pot_enable_s
{
  uint8_t wiper;                 /* Wiper index */
  bool    enable;                /* false forces wiper into shutdown */
};

/* Non-volatile transfer for POTIOC_STORE_WIPER and POTIOC_RECALL_WIPER */

struct pot_nv_s
{
  uint8_t wiper;                 /* Wiper index */
  uint8_t slot;                  /* Non-volatile slot index */
};

struct pot_dev_s;
struct pot_ops_s
{
  /* Configure the potentiometer. This method is called the first time
   * that the potentiometer device is opened.
   */

  CODE int (*po_setup)(FAR struct pot_dev_s *dev);

  /* Disable the potentiometer. This method is called when the
   * potentiometer device is closed. This method reverses the operation
   * of the setup method.
   */

  CODE void (*po_shutdown)(FAR struct pot_dev_s *dev);

  /* Set a wiper value. Optional. */

  CODE int (*po_setwiper)(FAR struct pot_dev_s *dev, uint8_t wiper,
                          uint32_t val);

  /* Get a wiper value. Optional. */

  CODE int (*po_getwiper)(FAR struct pot_dev_s *dev, uint8_t wiper,
                          FAR uint32_t *val);

  /* Move a wiper by a number of steps, negative moves down. Optional. */

  CODE int (*po_move)(FAR struct pot_dev_s *dev, uint8_t wiper,
                      int32_t steps);

  /* Enable or force a wiper into shutdown. Optional. */

  CODE int (*po_enable)(FAR struct pot_dev_s *dev, uint8_t wiper,
                        bool enable);

  /* Store a wiper value to a non-volatile slot. Optional. */

  CODE int (*po_store)(FAR struct pot_dev_s *dev, uint8_t wiper,
                       uint8_t slot);

  /* Recall a wiper value from a non-volatile slot. Optional. */

  CODE int (*po_recall)(FAR struct pot_dev_s *dev, uint8_t wiper,
                        uint8_t slot);

  /* Lower-half logic may support chip-specific ioctl commands */

  CODE int (*po_ioctl)(FAR struct pot_dev_s *dev, int cmd,
                       unsigned long arg);
};

struct pot_dev_s
{
  /* Fields managed by common upper half potentiometer logic */

  uint8_t                    pd_ocount;    /* The number of times the device
                                            * has been opened */
  mutex_t                    pd_closelock; /* Locks out new opens while close
                                            * is in progress */
  mutex_t                    pd_lock;      /* Serializes ioctl access */

  /* Fields provided by lower half potentiometer logic */

  uint8_t                    pd_nwipers;   /* Number of wipers */
  uint32_t                   pd_max;       /* Wiper full scale value */
  uint32_t                   pd_rab;       /* A-B resistance in ohms,
                                            * 0 if unknown */
  FAR const struct pot_ops_s *pd_ops;      /* Arch-specific operations */
  FAR void                   *pd_priv;     /* Used by the arch-specific
                                            * logic */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Name: pot_register
 *
 * Description:
 *   Register a digital potentiometer driver.
 *
 * Input Parameters:
 *   path - The full path to the driver to register, e.g. "/dev/pot0"
 *   dev  - An instance of the device-specific potentiometer interface
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pot_register(FAR const char *path, FAR struct pot_dev_s *dev);

/****************************************************************************
 * Name: mcp445x_initialize
 *
 * Description:
 *   Initialize a MCP445X potentiometer.
 *
 * Input Parameters:
 *   i2c  - An instance of the I2C interface to communicate with the device
 *   addr - The I2C address of the MCP445X
 *   rab  - Terminal A-B resistance in ohms, 0 if unknown
 *
 * Returned Value:
 *   Valid MCP445X device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_POT_MCP445X
struct i2c_master_s;
FAR struct pot_dev_s *mcp445x_initialize(FAR struct i2c_master_s *i2c,
                                         uint8_t addr, uint32_t rab);
#endif

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_ANALOG_POT_H */

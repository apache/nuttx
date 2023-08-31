/****************************************************************************
 * include/nuttx/net/ksz9477.h
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

#ifndef __INCLUDE_NUTTX_NET_KSZ9477_H
#define __INCLUDE_NUTTX_NET_KSZ9477_H

#ifdef CONFIG_NET_KSZ9477

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  KSZ9477_PORT_NONE = -1,
  KSZ9477_PORT_GLOBAL = 0,
  KSZ9477_PORT_PHY1 = 1,
  KSZ9477_PORT_PHY2 = 2,
  KSZ9477_PORT_PHY3 = 3,
  KSZ9477_PORT_PHY4 = 4,
  KSZ9477_PORT_PHY5 = 5,
  KSZ9477_PORT_RMII = 6,
  KSZ9477_PORT_SGMII = 7,
} ksz9477_port_t;

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ksz9477_i2c_init
 *
 * Description:
 *   Switches the ksz9477's SGMII port into PHY mode and sets the
 *   default settings to work directly with an external MAC
 *
 * Input Parameters:
 *   i2c_bus: Management i2c bus
 *   master_port: The port of the switch connected to host MAC
 *
 * Returned Value:
 *   OK or ERROR
 *
 ****************************************************************************/

#ifdef CONFIG_NET_KSZ9477_I2C
int ksz9477_i2c_init(struct i2c_master_s *i2c_bus,
                     ksz9477_port_t master_port);
#else
#  error Only I2c interface currently supported
#endif

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_NET_KSZ9477 */
#endif /* __INCLUDE_NUTTX_NET_KSZ9477_H */

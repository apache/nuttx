/****************************************************************************
 * include/nuttx/sensors/ds18b20.h
 * Character driver for DS18B20 Digital Temperature Module.
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

#ifndef __INCLUDE_NUTTX_SENSORS_DS18B20_H
#define __INCLUDE_NUTTX_SENSORS_DS18B20_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_1WIRE) && defined(CONFIG_SENSORS_DS18B20)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DS18B20_DEVICE_FAMILY         (0x28)

#define DS18B20_CMD_CONVERTT          (0x44)
#define DS18B20_CMD_COPYSPAD          (0x48)
#define DS18B20_CMD_RECALL            (0xb8)
#define DS18B20_CMD_READPOWS          (0xb4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct onewire_master_s;

struct ds18b20_alarm_s
{
  int8_t thigh;           /* Upper alarm temperature */
  int8_t tlow;            /* Lower alarm temperature */
# ifdef CONFIG_SENSORS_DS18B20_POLL
  bool   wakeup;          /* Wakeup poll requests only when alarm detected */
# endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ds18b20_register
 *
 * Description:
 *   Register the DS18B20 character device.
 *
 * Input Parameters:
 *   devno   - The user specifies device number, from 0.
 *   onewire - An instance of the 1wire interface to communicate with DS18B20
 *             sensor.
 *   romcode - The ROM code of the DS18B20.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 ****************************************************************************/

int ds18b20_register(int devno, FAR struct onewire_master_s *dev,
                     uint64_t romcode);
#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_1WIRE && CONFIG_SENSORS_DS18B20 */
#endif /* __INCLUDE_NUTTX_SENSORS_DS18B20_H */

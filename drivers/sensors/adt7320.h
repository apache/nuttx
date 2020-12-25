/****************************************************************************
 * drivers/sensors/adt7320.h
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef __DRIVERS_SENSORS_ADT7320_H
#define __DRIVERS_SENSORS_ADT7320_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADT7320_STAT_REG    0x00
#define ADT7320_CONF_REG    0x01
#define ADT7320_TEMP_REG    0x02
#define ADT7320_ID_REG      0x03
#define ADT7320_TCRIT_REG   0x04
#define ADT7320_THYST_REG   0x05
#define ADT7320_THIGH_REG   0x06
#define ADT7320_TLOW_REG    0x07

#define ADT7320_ID          0xc3

#endif /* __DRIVERS_SENSORS_ADT7320_H */

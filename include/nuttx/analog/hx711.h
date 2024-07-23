/****************************************************************************
 * include/nuttx/analog/hx711.h
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HX711_MAX_AVG_SAMPLES 225

/* ioctl requests ***********************************************************/

/* Set how many samples to read from hx711 to get a single averaged value.
 * Minimum value is 1. To prevent possible integer overflow, maximum value
 * is HX711_MAX_AVG_SAMPLES.
 */

#define HX711_SET_AVERAGE 0

/* Set channel to use for next read() operation. Channels 'a' and 'b'
 * are available. Specify channel as 'a' character (0x61 hex)
 */

#define HX711_SET_CHANNEL 1

/* Set gain to use for next read() operation. Channel 'b' only supports
 * gain of 32, and channel 'a' supports gain 128 and 64
 */

#define HX711_SET_GAIN 2

/* Set what value coresponds to 1 unit. Takes integer.
 * If set to 0 (default) driver will return raw readings from
 * hx711 instead of calculated units.
 */

#define HX711_SET_VAL_PER_UNIT 3

/* Depending on tensometer position, value will go higher or lower
 * (into negative values) when mass increases. If your sign does
 * not match, it can be changed by calling this.
 *  1 - no sign change (default)
 * -1 - sign will be changed
 */

#define HX711_SET_SIGN 4

/* ioctl get functions */

/* Get current average, pass pointer to unsigned int type */

#define HX711_GET_AVERAGE 100

/* Get current channel, pass pointer to single char */

#define HX711_GET_CHANNEL 101

/* Get current gain, pass pointer to single unsignedchar */

#define HX711_GET_GAIN 102

/* Get current value per unit */

#define HX711_GET_VAL_PER_UNIT 103

/* Tare the scale. Accepts int value with desired precision.
 * If HX711_VAL_PER_UNIT was set earlier, you should pass value
 * in units, otherwise you need to pass raw value as read from hx711.
 * Takes pointer to a float value.
 */

#define HX711_TARE 200

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* hx711 exposes 2 pins for communication. One is for data reading, and
 * second one is clock signal. This is similar to i2c but hx711 uses custom
 * protocol that is not compatible with i2c in any way.
 *
 * Platform code should provide these functions to manipulate these GPIOs
 */

struct hx711_lower_s
{
  /**************************************************************************
   * Name: clock_set
   *
   * Description:
   *   Sets underlying GPIO pin according to val.
   *
   * Input Parameters:
   *   val - set GPIO pin high (1) or low (0)
   *   minor - hx711 device being manipulated
   *
   * Returned Value:
   *   OK on success, or negated errno on failure
   *
   **************************************************************************/

  CODE int (*clock_set)(unsigned char minor, int val);

  /**************************************************************************
   * Name: data_read
   *
   * Description:
   *   Reads current value of data GPIO pin.
   *
   * Input Parameters:
   *   minor - hx711 device being manipulated
   *
   * Returned Value:
   *   For success, return 0 when GPIO is low, 1 when GPIO is high
   *   or negated errno on failure.
   *
   **************************************************************************/

  CODE int (*data_read)(unsigned char minor);

  /**************************************************************************
   * Name: cleanup
   *
   * Description:
   *   This function is called when last instance of minor is closed and
   *   unlinked from fs so that hx711 minor instance is no longer available.
   *   Platform should free all resources it allocated to register the
   *   device.
   *
   *   This function does not have to be set, if there is nothing to clean.
   *
   * Input Parameters:
   *   minor - hx711 instance being destroyed
   *
   **************************************************************************/

  CODE void (*cleanup)(unsigned char minor);

  /**************************************************************************
   * Name: data_irq
   *
   * Description:
   *   Setup (or tear down when handler is NULL) interrupt when data line
   *   goes from HIGH to LOW state (falling edge).
   *
   *   hx711 is slow, on internal oscillator and RATE=0 it takes 100ms to
   *   sample a single reading. To avoid hogging CPU polling for data to
   *   go down, driver will install interrupt handler before reading.
   *   Once interrupt is received, driver will disable the handler.
   *
   * Input Parameters:
   *   minor - hx711 device being manipulated
   *   handler - function interrupt should call
   *   arg - private data for handler, should be passed to handler
   *
   * Returned Value:
   *   On successfull interrupt initialization 0 should be returned,
   *   when there was failure initializing interrupt -1 shall be returned.
   *
   **************************************************************************/

  CODE int (*data_irq)(unsigned char minor, xcpt_t handler, void *arg);
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: hx711_register
 *
 * Description:
 *   Register new hx711 device in /dev/hx711_%d. Multiple hx711 can be
 *   supported by providing different minor number. When driver calls
 *   platform specific function, minor number is passed back, so platform
 *   can know which hx711 is manipulated.
 *
 * Input Parameters:
 *   minor - unique number identifying hx711 chip.
 *   lower - provided by platform code to manipulate hx711 with platform
 *           dependant functions>
 *
 * Returned Value:
 *   OK on success, or negated errno on failure
 *
 ****************************************************************************/

int hx711_register(unsigned char minor, FAR struct hx711_lower_s *lower);

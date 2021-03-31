/****************************************************************************
 * include/nuttx/sensors/cluster_driver.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_CLUSTER_DRIVER_H
#define __INCLUDE_NUTTX_SENSORS_CLUSTER_DRIVER_H

/* Definitions for the cluster driver interface:
 *
 * The sensor cluster driver interface is created to permit high performance
 * collection and processing of sensor data. Normally when high performance
 * sensor data collection is required, data is needed from multiple sensors
 * and the collection/processing task needs to run as a "higher than user
 * priority" priority level. This mechanism is commonly supported by writing
 * the collection/processing task as a kernel driver worker thread that can
 * access multiple sensors.
 *
 * A clean way to implement this is to implement this collection/processing
 * mechanism is as a driver that has efficient kernel-to-kernel access to
 * other sensor leaf-drivers that support the cluster driver interface.
 * (This documentation describes the "cluster driver interface" from the
 * perspective of a "cluster driver interface" enabled leaf driver.)
 *
 * The file_operations interface typically deals with the caller's parameters
 * and buffers being in user space, while the driver's code, variables, and
 * buffers are in kernel space. There is also a layer of kernel code between
 *  the caller and driver that deals with security and assists helps with the
 * driver's access of user space memory. So the file_operations interface
 * is not appropriate for driver-to-driver communication.
 *
 * Since the driver registration function is not part of the file_operations
 * interface and is permitted to be called from a kernel task, this function
 * is reused. But rather than being called by the board initialization
 * function, the cluster drivers registration function is called from the
 * board initialization function; and the cluster drivers registration
 * function calls the leaf driver's registration function.
 *
 * To be "cluster driver interface" enabled the leaf driver's registration
 * function must communicate the leaf driver's instance back to the cluster
 * driver's registration function. This is done by storing the leaf
 * driver's instance handle into the caller provided config structure.
 * This handle is provided as a parameter in cluster driver interface calls.
 *
 * dvr_open(): Reserve this sensor.
 *
 * dvr_close(): Places the sensor into low power standby mode, frees driver
 *   resources associated with the sensor, and release the reservation
 *   . of the sensor.
 *
 * To perform I/O to a sensor, the cluster driver needs...
 *
 * > A pointer to the spi instance (struct spi_dev_s *). The cluster driver
 *   has this pointer, because it provides it to the leaf driver as an input
 *   parameter to the leaf driver's registration function.
 * > The chip select gpio pin identifier for the sensor device. The cluster
 *   driver knows this identifier,because it provides it to the leaf driver
 *   as an input parameter to the leaf driver's registration function.
 * > A pointer to the leaf driver instance. The leaf driver communicates this
 *   pointer to the cluster driver by storing it into the config struct
 *   leaf_handle (struct spi_dev_s *) field that is also passed as an in/out
 *   parameter in the leaf driver's registration function.
 *
 * With the above information, the sensor cluster driver may efficiently
 * access multiple sensors and aggregate their data.
 *
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Cluster driver operations interface */

struct sensor_cluster_operations_s
{
  CODE int (*driver_open)(FAR void *instance_handle, int32_t arg);
  CODE int (*driver_close)(FAR void *instance_handle, int32_t arg);
  CODE ssize_t (*driver_read)(FAR void *instance_handle, FAR char *buffer,
            size_t buflen);
  CODE ssize_t (*driver_write)(FAR void *instance_handle,
            FAR const char *buffer, size_t buflen);
  CODE off_t (*driver_seek)(FAR void *instance_handle, off_t offset,
            int whence);
  CODE int (*driver_ioctl)(FAR void *instance_handle, int cmd,
            unsigned long arg);
  CODE int (*driver_suspend)(FAR void *instance_handle, int32_t arg);
  CODE int (*driver_resume)(FAR void *instance_handle, int32_t arg);
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
 * Name: xxxxxx_register   <-- for a leaf driver.
 *
 * Description:
 *   Example of a driver register function for a "sensor leaf driver" that
 *   can be controlled by a "sensor cluster driver".  The specific format of
 *   the registration function will vary based on the characteristics of the
 *   driver. The below is an example, which describes common aspects of the
 *   function and and its parameters.
 *
 *   Normally, sensor driver register functions are called by the board
 *   initialization code. In the case of a "sensor cluster driver", the board
 *   initialization code calls the register function of the "sensor cluster
 *   driver"; and the "sensor cluster driver" calls the register function of
 *   each of the sensor (leaf) drivers that it will be controlling.
 *
 * Input Parameters:
 *   devpath  - The full path to the leaf driver to register. E.g.,
 *           "/dev/acl0"
 *   spi      - An instance of the SPI interface to use to communicate with
 *              the leaf driver. Or, it could be the I2C driver instance if
 *              the sensor is on an I2C bus.
 *   config   - configuration struct for the sensor leaf driver.
 *              This struct is defined in the leaf driver's xxxxxx.h file.
 *
 *  For a SPI sensor, this structure must contain:
 *
 *   int spi_devid;   The spi device id, which is used by NuttX to
 *        select/deselect the device. This is usually some
 *        type of reference to the chip_select gpio pin.
 *   FAR void *leaf_handle; The handle to the leaf driver instance.
 *        This is an opaque handle that provided by the
 *        leaf driver's register function.
 *        It is passed as a parameter to the leaf driver's
 *        driver_open() and driver_close() functions.
 *
 *  For an I2C sensor, this struct must contain:
 *
 *   int i2c_devid;   The i2c device id, which is used by NuttX to
 *        address the sensor device on an I2C bus..
 *   FAR void *leaf_handle; The handle to the leaf driver instance.
 *        This is an opaque handle that provided by the
 *        leaf driver's register function.
 *        It is passed as a parameter to the leaf driver's
 *        driver_open() and driver_close() functions.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * int xxxxxx_register(FAR const char *devpath,
 *                     FAR struct spi_dev_s *spi,
 *                     FAR struct xxxxxx_config_s *config);
 *
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_CLUSTER_DRIVER_H */

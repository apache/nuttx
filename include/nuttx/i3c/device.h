/****************************************************************************
 * include/nuttx/i3c/device.h
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

#ifndef __INCLUDE_NUTTX_I3C_DEV_H
#define __INCLUDE_NUTTX_I3C_DEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/list.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/i3c/i3c_driver.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define I3C_PID_MANUF_ID(pid)           (((pid) & I3C_GENMASK_ULL(47, 33)) >> 33)
#define I3C_PID_RND_LOWER_32BITS(pid)   (!!((pid) & BIT_ULL(32)))
#define I3C_PID_RND_VAL(pid)            ((pid) & I3C_GENMASK_ULL(31, 0))
#define I3C_PID_PART_ID(pid)            (((pid) & I3C_GENMASK_ULL(31, 16)) >> 16)
#define I3C_PID_INSTANCE_ID(pid)        (((pid) & I3C_GENMASK_ULL(15, 12)) >> 12)
#define I3C_PID_EXTRA_INFO(pid)         ((pid) & I3C_GENMASK_ULL(11, 0))

#define I3C_BCR_DEVICE_ROLE(bcr)        ((bcr) & I3C_GENMASK(7, 6))
#define I3C_BCR_I3C_SLAVE               (0 << 6)
#define I3C_BCR_I3C_MASTER              (1 << 6)
#define I3C_BCR_HDR_CAP                 I3C_BIT(5)
#define I3C_BCR_BRIDGE                  I3C_BIT(4)
#define I3C_BCR_OFFLINE_CAP             I3C_BIT(3)
#define I3C_BCR_IBI_PAYLOAD             I3C_BIT(2)
#define I3C_BCR_IBI_REQ_CAP             I3C_BIT(1)
#define I3C_BCR_MAX_DATA_SPEED_LIM      I3C_BIT(0)

/* i3c */

#define I3C_MATCH_DCR                   0x1
#define I3C_MATCH_MANUF                 0x2
#define I3C_MATCH_PART                  0x4
#define I3C_MATCH_EXTRA_INFO            0x8

/* Bit definitions for the flags field in struct i3c_priv_xfer
 *
 * I3C_MASTER_READ macro indicates a read data from slave to master
 * I3C_MASTER_WRITE macro indicates a write data from slave to master
 */

#define I3C_MASTER_READ                 0x01
#define I3C_MASTER_WRITE                0x00

/* These macros should be used to i3c_device_id entries. */

#define I3C_MATCH_MANUF_AND_PART (I3C_MATCH_MANUF | I3C_MATCH_PART)

#define I3C_DEVICE(_manufid, _partid, _drvdata) \
  {   \
    .match_flags = I3C_MATCH_MANUF_AND_PART,    \
    .manuf_id = _manufid,       \
    .part_id = _partid,         \
    .data = _drvdata,           \
  }

#define I3C_DEVICE_EXTRA_INFO(_manufid, _partid, _info, _drvdata)  \
  {   \
    .match_flags = I3C_MATCH_MANUF_AND_PART |  \
             I3C_MATCH_EXTRA_INFO,             \
    .manuf_id = _manufid,        \
    .part_id = _partid,          \
    .extra_info = _info,         \
    .data = _drvdata,            \
  }

#define I3C_CLASS(_dcr, _drvdata)        \
  {   \
    .match_flags = I3C_MATCH_DCR,        \
    .dcr = _dcr,           \
  }

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i3c_device;
struct i3c_ccc_cmd;

struct i3c_device_id
{
  uint8_t match_flags;
  uint8_t dcr;
  uint16_t manuf_id;
  uint16_t part_id;
  uint16_t extra_info;

  FAR const void *data;
};

/* enum i3c_error_code - I3C error codes
 *
 * These are the standard error codes as defined by the I3C specification.
 * When -EIO is returned by the i3c_device_do_priv_xfers() or
 * i3c_device_send_hdr_cmds() one can check the error code in
 * &struct_i3c_priv_xfer.err or &struct i3c_hdr_cmd.err to get a better
 * idea of what went wrong.
 *
 * @I3C_ERROR_UNKNOWN: unknown error, usually means the error is not I3C
 *      related
 * @I3C_ERROR_M0: M0 error
 * @I3C_ERROR_M1: M1 error
 * @I3C_ERROR_M2: M2 error
 */

enum i3c_error_code
{
  I3C_ERROR_UNKNOWN = 0,
  I3C_ERROR_M0 = 1,
  I3C_ERROR_M1,
  I3C_ERROR_M2,
};

/* enum i3c_hdr_mode - HDR mode ids
 * @I3C_HDR_DDR: DDR mode
 * @I3C_HDR_TSP: TSP mode
 * @I3C_HDR_TSL: TSL mode
 */

enum i3c_hdr_mode
{
  I3C_HDR_DDR,
  I3C_HDR_TSP,
  I3C_HDR_TSL,
};

/* struct i3c_priv_xfer - I3C SDR private transfer
 * @flags: encodes the transfer direction. bit 0 indicates read or write,
 *      set 1 for read, set 0 for write
 * @len: transfer length in bytes of the transfer
 * @data: input/output buffer
 * @data.in: input buffer. Must point to a DMA-able buffer
 * @data.out: output buffer. Must point to a DMA-able buffer
 * @err: I3C error code
 */

struct i3c_priv_xfer
{
  uint16_t flags;
  uint16_t len;
  union
  {
    FAR void *in;
    FAR const void *out;
  } data;
  enum i3c_error_code err;
};

/* enum i3c_dcr - I3C DCR values
 * @I3C_DCR_GENERIC_DEVICE: generic I3C device
 */

enum i3c_dcr
{
  I3C_DCR_GENERIC_DEVICE = 0,
};

/* struct i3c_device_info - I3C device information
 * @pid: Provisional ID
 * @bcr: Bus Characteristic Register
 * @dcr: Device Characteristic Register
 * @static_addr: static/I2C address
 * @dyn_addr: dynamic address
 * @hdr_cap: supported HDR modes
 * @max_read_ds: max read speed information
 * @max_write_ds: max write speed information
 * @max_ibi_len: max IBI payload length
 * @max_read_turnaround: max read turn-around time in micro-seconds
 * @max_read_len: max private SDR read length in bytes
 * @max_write_len: max private SDR write length in bytes
 *
 * These are all basic information that should be advertised by an I3C
 * device.
 * Some of them are optional depending on the device type and device
 * capabilities.
 * For each I3C slave attached to a master with
 * i3c_master_add_i3c_dev_locked(), the core will send the relevant CCC
 * command to retrieve these data.
 */

struct i3c_device_info
{
  uint64_t pid;
  uint8_t bcr;
  uint8_t dcr;
  uint8_t static_addr;
  uint8_t dyn_addr;
  uint8_t hdr_cap;
  uint8_t max_read_ds;
  uint8_t max_write_ds;
  uint8_t max_ibi_len;
  uint32_t max_read_turnaround;
  uint16_t max_read_len;
  uint16_t max_write_len;
};

/* I3C device internals are kept hidden from I3C device users. It's just
 * simpler to refactor things when everything goes through getter/setters,
 * and I3C device drivers should not have to worry about internal
 * representation anyway.
 */

struct i3c_ibi_payload
{
  unsigned int len;
  FAR const void *data;
};

/* struct i3c_ibi_setup - IBI setup object
 * @max_payload_len: maximum length of the payload associated to an IBI.
 *         If one IBI appears to have a payload that is bigger than this
 *         number, the IBI will be rejected.
 * @num_slots: number of pre-allocated IBI slots. This should be chosen so
 *         that the system never runs out of IBI slots, otherwise you'll
 *         lose IBIs.
 * @handler: IBI handler, every time an IBI is received. This handler is
 *       called in a workqueue context. It is allowed to sleep and send new
 *       messages on the bus, though it's recommended to keep the
 *       processing done there as fast as possible to avoid delaying
 *       processing of other queued on the same workqueue.
 *
 * Temporary structure used to pass information to i3c_device_request_ibi().
 * This object can be allocated on the stack since i3c_device_request_ibi()
 * copies every bit of information and do not use it after
 * i3c_device_request_ibi() has returned.
 */

struct i3c_ibi_setup
{
  unsigned int max_payload_len;
  unsigned int num_slots;
  CODE void (*handler)(FAR struct i3c_device *dev,
                       FAR const struct i3c_ibi_payload *payload);
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: i3c_device_do_priv_xfers
 *
 * Description:
 *   Do I3C SDR private transfers directed to a specific device.
 *
 *   Initiate one or several private SDR transfers with @dev.
 *
 *   This function can sleep and thus cannot be called in atomic context.
 *
 * Input Parameters:
 *   desc   - An I3C device descriptor will be used for
 *   xfers  - Array of transfers
 *   nxfers - Number of transfers
 *
 * Returned Value:
 *   0 in case of success, a negative error core otherwise.
 *
 ****************************************************************************/

int i3c_device_do_priv_xfers(FAR const struct i3c_device *dev,
                             FAR struct i3c_priv_xfer *xfers,
                             int nxfers);

/****************************************************************************
 * Name: i3c_device_get_info
 *   get I3C device information, Retrieve I3C dev info.
 *
 * Input Parameters:
 *   dev  - A device we want information on.
 *   info - The information object to fill in.
 *
 ****************************************************************************/

void i3c_device_get_info(FAR const struct i3c_device *dev,
                         FAR struct i3c_device_info *info);

/****************************************************************************
 * Name: i3c_device_disable_ibi
 *
 * Description:
 *   Disable IBIs coming from a specific device.
 *
 *   This function disable IBIs coming from a specific device and wait for
 *   all pending IBIs to be processed.
 *
 * Input Parameters:
 *   dev - A device on which IBIs should be disabled
 *
 * Returned Value:
 *   0 in case of success, a negative error core otherwise.
 *
 ****************************************************************************/

int i3c_device_disable_ibi(FAR const struct i3c_device *dev);

/****************************************************************************
 * Name: i3c_device_enable_ibi
 *
 * Description:
 *   Enable IBIs coming from a specific device.
 *
 *   This function enable IBIs coming from a specific device and wait for
 *   all pending IBIs to be processed. This should be called on a device
 *   where i3c_device_request_ibi() has succeeded.
 *
 *   Note that IBIs from this device might be received before this function
 *   returns to its caller.
 *
 * Input Parameters:
 *   dev - A device on which IBIs should be enabled
 *
 * Returned Value:
 *   0 in case of success, a negative error core otherwise.
 *
 ****************************************************************************/

int i3c_device_enable_ibi(FAR const struct i3c_device *dev);

/****************************************************************************
 * Name: i3c_device_request_ibi
 *
 * Description:
 *   Request an IBI.
 *
 *   This function is responsible for pre-allocating all resources needed to
 *   process IBIs coming from @dev. When this function returns, the IBI is
 *   not enabled until i3c_device_enable_ibi() is called.
 *
 * Input Parameters:
 *   dev - An I3C device descriptor will be used for.
 *   req - setup requested for this IBI.
 *
 * Returned Value:
 *   0 in case of success, a negative error core otherwise.
 *
 ****************************************************************************/

int i3c_device_request_ibi(FAR const struct i3c_device *dev,
                           FAR const struct i3c_ibi_setup *req);

/****************************************************************************
 * Name: i3c_device_free_ibi
 *
 * Description:
 *   Free all resources needed for IBI handling.
 *
 *   This function is responsible for de-allocating resources previously
 *   allocated by i3c_device_request_ibi(). It should be called after
 *   disabling IBIs with i3c_device_disable_ibi().
 *
 * Input Parameters:
 *   dev - Device on which you want to release IBI resources
 *
 ****************************************************************************/

void i3c_device_free_ibi(FAR const struct i3c_device *dev);

/****************************************************************************
 * Name: i3c_master_i2c_attach
 *
 * Description:
 *   Config an i2c device address to controller driver.
 *
 * Input Parameters:
 *   master - The master used to get i3c_device on the bus
 *   config - An i2c device information to add
 *
 * Returned Value:
 *  Return 0 if success, otherwise a negative number.
 ****************************************************************************/

int i3c_master_i2c_attach(FAR struct i3c_master_controller *master,
                          FAR struct i2c_config_s *config);

/****************************************************************************
 * Name: i3c_master_detach_i2c_dev
 *
 * Description:
 *   Delete an i2c device address in controller driver. .
 *
 * Input Parameters:
 *   master - The master used to get i3c_device on the bus
 *   config - An i2c device information to delete
 ****************************************************************************/

void i3c_master_detach_i2c_dev(FAR struct i3c_master_controller *master,
                               FAR struct i2c_config_s *config);

/****************************************************************************
 * Name: i3c_master_find_i3c_dev
 *
 * Description:
 *   This function is used to be find a i3c_device address by master handle
 *   and provisional ID.
 *
 * Input Parameters:
 *   master - The master used to get i3c_device on the bus
 *   id     - An instance of i3c_device_id, include manufid,partid and so on.
 * Returned Value:
 *   Struct i3c_device var in case of success, NULL otherwise.
 ****************************************************************************/

FAR const struct i3c_device *i3c_master_find_i3c_dev(
                              FAR struct i3c_master_controller *master,
                              FAR const struct i3c_device_id *id);

/****************************************************************************
 * Name: i3c_device_send_ccc_cmd
 *
 * Description:
 *   This function is used to send a common ccc command.
 *
 * Input Parameters:
 *   dev - An I3C device descriptor will be used for
 *   cmd - The buf of ccc commands to transfer, only one frame at a time
 *
 * Returned Value:
 *   0 or positive if Success, nagative otherwise.
 ****************************************************************************/

int i3c_device_send_ccc_cmd(FAR const struct i3c_device *dev,
                            FAR struct i3c_ccc_cmd *cmd);

#endif /* __INCLUDE_NUTTX_I3C_DEV_H */

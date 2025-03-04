/****************************************************************************
 * include/nuttx/i3c/master.h
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

#ifndef __INCLUDE_NUTTX_I3C_MASTER_H
#define __INCLUDE_NUTTX_I3C_MASTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/atomic.h>
#include <nuttx/wqueue.h>
#include <nuttx/list.h>
#include <nuttx/mutex.h>
#include <nuttx/spinlock.h>
#include <nuttx/bits.h>

#include <nuttx/i3c/device.h>
#include <nuttx/i3c/ccc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/i3c/i3c_driver.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define I3C_BIT(n)              (1ul << (n))
#define I3C_BITS_PER_LONG       BITS_PER_LONG
#define I3C_BITS_PER_LONG_LONG  BITS_PER_LONG_LONG
#define I3C_GENMASK(h, l)       GENMASK(h, l)
#define I3C_GENMASK_ULL(h, l)   GENMASK_ULL(h, l)

#define I3C_HOT_JOIN_ADDR       0x2
#define I3C_BROADCAST_ADDR      0x7e
#define I3C_MAX_ADDR            I3C_GENMASK(6, 0)

#define I3C_LVR_I2C_INDEX_MASK  I3C_GENMASK(7, 5)
#define I3C_LVR_I2C_INDEX(x)    ((x) << 5)
#define I3C_LVR_I2C_FM_MODE     I3C_BIT(4)

#define I3C_I2C_MAX_ADDR        I3C_GENMASK(6, 0)

/* use ten bit chip address */

#define I3C_I2C_CLIENT_TEN      0x10

/* The I3C specification says the maximum number of devices connected on the
 * bus is 11, but this number depends on external parameters like trace
 * length, capacitive load per Device, and the types of Devices present on
 * the Bus.
 * I3C master can also have limitations, so this number is just here as a
 * reference and should be adjusted on a per-controller/per-board basis.
 */

#define I3C_BUS_MAX_DEVS              11

#define I3C_BUS_MAX_I3C_SCL_RATE      12900000
#define I3C_BUS_TYP_I3C_SCL_RATE      12500000
#define I3C_BUS_I2C_FM_PLUS_SCL_RATE  1000000
#define I3C_BUS_I2C_FM_SCL_RATE       400000
#define I3C_BUS_TLOW_OD_MIN_NS        200

/* i3c_bus_for_each_i3cdev() - iterate over all I3C devices present on the
 *     bus
 * @bus: the I3C bus
 * @dev: and I3C device descriptor pointer updated to point to the current
 *     slot
 *   at each iteration of the loop
 *
 * Iterate over all I3C devs present on the bus.
 */

#define i3c_bus_for_each_i3cdev(bus, dev)       \
  list_for_every_entry(&(bus)->devs.i3c, dev, struct i3c_dev_desc, common.node)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i3c_master_controller;
struct i3c_bus;
struct i3c_device;
struct i3c_dev_desc;
struct i3c_device_info;
struct i3c_generic_ibi_pool;

/* struct i3c_i2c_dev_desc - Common part of the I3C/I2C device descriptor
 * @node: node element used to insert the slot into the I2C or I3C device
 *      list
 * @master: I3C master that instantiated this device. Will be used to do
 *      I2C/I3C transfers
 * @master_priv: master private data assigned to the device. Can be used
 * to add master specific information
 *
 * This structure is describing common I3C/I2C dev information.
 */

struct i3c_i2c_dev_desc
{
  struct list_node node;
  FAR struct i3c_master_controller *master;
  FAR void *master_priv;
};

struct i2c_board_info
{
  uint32_t i2c_scl;
  uint16_t addr;
  uint16_t addrlen;
};

/* struct i2c_dev_boardinfo - I2C device board information
 * @node: used to insert the boardinfo object in the I2C boardinfo list
 * @base: regular I2C board information
 * @lvr: LVR (Legacy Virtual Register) needed by the I3C core to know about
 *   the I2C device limitations
 *
 * This structure is used to attach board-level information to an I2C device.
 * Each I2C device connected on the I3C bus should have one.
 */

struct i2c_dev_boardinfo
{
  struct list_node node;
  struct i2c_board_info base;
  unsigned char lvr;
};

/* struct i2c_dev_desc - I2C device descriptor
 * @common: common part of the I2C device descriptor
 * @boardinfo: pointer to the boardinfo attached to this I2C device
 * @addr: I2C device address
 * @lvr: LVR (Legacy Virtual Register) needed by the I3C core to know about
 *   the I2C device limitations
 *
 * Each I2C device connected on the bus will have an i2c_dev_desc.
 * This object is created by the core and later attached to the controller
 * using &struct_i3c_master_controller->ops->attach_i2c_dev().
 *
 * &struct_i2c_dev_desc is the internal representation of an I2C device
 * connected on an I3C bus. This object is also passed to all
 * &struct_i3c_master_controller_ops hooks.
 */

struct i2c_dev_desc
{
  struct i3c_i2c_dev_desc common;
  FAR const struct i2c_dev_boardinfo *boardinfo;
  uint16_t addr;
  unsigned char lvr;
};

/* struct i3c_ibi_slot - I3C IBI (In-Band Interrupt) slot
 * @work: work associated to this slot. The IBI handler will be called
 * from there
 * @dev: the I3C device that has generated this IBI
 * @len: length of the payload associated to this IBI
 * @data: payload buffer
 *
 * An IBI slot is an object pre-allocated by the controller and used when
 * an IBI comes in.
 * Every time an IBI comes in, the I3C master driver should find a free IBI
 * slot in its IBI slot pool, retrieve the IBI payload and queue the IBI
 * using i3c_master_queue_ibi().
 *
 * How IBI slots are allocated is left to the I3C master driver, though,
 * for simple kmalloc-based allocation, the generic IBI slot pool can be
 * used.
 */

struct i3c_ibi_slot
{
  struct work_s work;
  FAR struct i3c_dev_desc *dev;
  unsigned int len;
  FAR void *data;
};

/* struct i3c_device_ibi_info - IBI information attached to a specific device
 * @all_ibis_handled: used to be informed when no more IBIs are waiting to be
 *       processed. Used by i3c_device_disable_ibi() to wait for
 *       all IBIs to be dequeued
 * @pending_ibis: count the number of pending IBIs. Each pending IBI has its
 *       work element queued to the controller workqueue
 * @max_payload_len: maximum payload length for an IBI coming from this
 *       device.
 *       this value is specified when calling
 *       i3c_device_request_ibi() and should not change at run
 *       time. All messages IBIs exceeding this limit should be
 *       rejected by the master
 * @num_slots: number of IBI slots reserved for this device
 * @enabled: reflect the IBI status
 * @handler: IBI handler specified at i3c_device_request_ibi() call time.
 *       This handler will be called from the controller workqueue, and as
 *       such is allowed to sleep (though it is recommended to process the
 *       IBI as fast as possible to not stall processing of other IBIs queued
 *       on the same workqueue).
 *       New I3C messages can be sent from the IBI handler
 *
 * The &struct_i3c_device_ibi_info object is allocated when
 * i3c_device_request_ibi() is called and attached to a specific device. This
 * object is here to manage IBIs coming from a specific I3C device.
 *
 * Note that this structure is the generic view of the IBI management
 * infrastructure. I3C master drivers may have their own internal
 * representation which they can associate to the device using
 * controller-private data.
 */

struct i3c_device_ibi_info
{
  sem_t all_ibis_handled;
  atomic_t pending_ibis;
  unsigned int max_payload_len;
  unsigned int num_slots;
  unsigned int enabled;
  CODE void (*handler)(FAR struct i3c_device *dev,
      const struct i3c_ibi_payload *payload);
};

/* struct i3c_dev_boardinfo - I3C device board information
 * @node: used to insert the boardinfo object in the I3C boardinfo list
 * @init_dyn_addr: initial dynamic address requested by the FW. We provide
 *    no guarantee that the device will end up using this address,
 *    but try our best to assign this specific address to the
 *    device
 * @static_addr: static address the I3C device listen on before it's been
 *    assigned a dynamic address by the master. Will be used during
 *    bus initialization to assign it a specific dynamic address
 *    before starting DAA (Dynamic Address Assignment)
 * @pid: I3C Provisional ID exposed by the device. This is a unique
 *    identifier that may be used to attach boardinfo to i3c_dev_desc when
 *    the device does not have a static address
 *
 * This structure is used to attach board-level information to an I3C device.
 * Not all I3C devices connected on the bus will have a boardinfo. It's only
 * needed if you want to attach extra resources to a device or assign it a
 * specific dynamic address.
 */

struct i3c_dev_boardinfo
{
  struct list_node node;
  unsigned char init_dyn_addr;
  unsigned char static_addr;
  uint64_t pid;
};

/* struct i3c_dev_desc - I3C device descriptor
 * @common: common part of the I3C device descriptor
 * @info: I3C device information. Will be automatically filled when you
 *   create your device with i3c_master_add_i3c_dev_locked()
 * @ibi_lock: lock used to protect the &struct_i3c_device->ibi
 * @ibi: IBI info attached to a device. Should be NULL until
 *   i3c_device_request_ibi() is called
 * @dev: pointer to the I3C device object exposed to I3C device drivers. This
 *   should never be accessed from I3C master controller drivers. Only core
 *   code should manipulate it in when updating the dev <-> desc link or
 *   when propagating IBI events to the driver
 * @boardinfo: pointer to the boardinfo attached to this I3C device
 *
 * Internal representation of an I3C device. This object is only used by the
 * core and passed to I3C master controller drivers when they're requested to
 * do some operations on the device.
 * The core maintains the link between the internal I3C dev descriptor and
 * the object exposed to the I3C device drivers (&struct_i3c_device).
 */

struct i3c_dev_desc
{
  struct i3c_i2c_dev_desc common;
  struct i3c_device_info info;
  mutex_t ibi_lock;
  FAR struct i3c_device_ibi_info *ibi;
  FAR struct i3c_device *dev;
  FAR const struct i3c_dev_boardinfo *boardinfo;
};

/* struct i3c_device - I3C device object
 * @desc: pointer to an i3c device descriptor object. This link is updated
 *   every time the I3C device is rediscovered with a different dynamic
 *   address assigned
 * @bus: I3C bus this device is attached to
 *
 * I3C device object exposed to I3C device drivers. The takes care of linking
 * this object to the relevant &struct_i3c_dev_desc one.
 * All I3C devs on the I3C bus are represented, including I3C masters. For
 * each of them, we have an instance of &struct i3c_device.
 */

struct i3c_device
{
  FAR struct i3c_dev_desc *desc;
  FAR struct i3c_bus *bus;
};

/* enum i3c_bus_mode - I3C bus mode
 * @I3C_BUS_MODE_PURE: only I3C devices are connected to the bus.
 *           No limitation expected
 * @I3C_BUS_MODE_MIXED_FAST: I2C devices with 50ns spike filter are present
 *           on the bus. The only impact in this mode is that the
 *           high SCL pulse has to stay below 50ns to trick I2C
 *           devices when transmitting I3C frames
 * @I3C_BUS_MODE_MIXED_LIMITED: I2C devices without 50ns spike filter are
 *        present on the bus. However they allow
 *        compliance up to the maximum SDR SCL clock
 *        frequency.
 * @I3C_BUS_MODE_MIXED_SLOW: I2C devices without 50ns spike filter are
 *           present on the bus
 */

enum i3c_bus_mode
{
  I3C_BUS_MODE_PURE,
  I3C_BUS_MODE_MIXED_FAST,
  I3C_BUS_MODE_MIXED_LIMITED,
  I3C_BUS_MODE_MIXED_SLOW,
};

/* enum i3c_addr_slot_status - I3C address slot status
 * @I3C_ADDR_SLOT_FREE: address is free
 * @I3C_ADDR_SLOT_RSVD: address is reserved
 * @I3C_ADDR_SLOT_I2C_DEV: address is assigned to an I2C device
 * @I3C_ADDR_SLOT_I3C_DEV: address is assigned to an I3C device
 * @I3C_ADDR_SLOT_STATUS_MASK: address slot mask
 *
 * On an I3C bus, addresses are assigned dynamically, and we need to know
 * which addresses are free to use and which ones are already assigned.
 *
 * Addresses marked as reserved are those reserved by the I3C protocol
 * (broadcast address, ...).
 */

enum i3c_addr_slot_status
{
  I3C_ADDR_SLOT_FREE,
  I3C_ADDR_SLOT_RSVD,
  I3C_ADDR_SLOT_I2C_DEV,
  I3C_ADDR_SLOT_I3C_DEV,
  I3C_ADDR_SLOT_STATUS_MASK = 3,
};

/* struct i3c_bus - I3C bus object
 * @cur_master: I3C master currently driving the bus. Since I3C is
 *    multi-master this can change over the time. Will be used to let
 *    a master know whether it needs to request bus ownership before
 *    sending a frame or not
 * @id: bus ID. Assigned by the framework when register the bus
 * @addrslots: a bitmap with 2-bits per-slot to encode the address status
 *    and ease the DAA (Dynamic Address Assignment) procedure (see
 *    &enum i3c_addr_slot_status)
 * @mode: bus mode (see &enum i3c_bus_mode)
 * @scl_rate.i3c: maximum rate for the clock signal when doing I3C SDR/priv
 *    transfers
 * @scl_rate.i2c: maximum rate for the clock signal when doing I2C transfers
 * @scl_rate: SCL signal rate for I3C and I2C mode
 * @devs.i3c: contains a list of I3C device descriptors representing I3C
 *    devices connected on the bus and successfully attached to the
 *    I3C master
 * @devs.i2c: contains a list of I2C device descriptors representing I2C
 *    devices connected on the bus and successfully attached to the
 *    I3C master
 * @devs: 2 lists containing all I3C/I2C devices connected to the bus
 * @lock: read/write lock on the bus. This is needed to protect against
 *    operations that have an impact on the whole bus and the devices
 *    connected to it. For example, when asking slaves to drop their
 *    dynamic address (RSTDAA CCC), we need to make sure no one is trying
 *    to send I3C frames to these devices.
 *    Note that this lock does not protect against concurrency between
 *    devices: several drivers can send different I3C/I2C frames through
 *    the same master in parallel. This is the responsibility of the
 *    master to guarantee that frames are actually sent sequentially and
 *    not interlaced
 *
 * The I3C bus is represented with its own object and not implicitly
 * described by the I3C master to cope with the multi-master functionality,
 * where one bus can be shared amongst several masters, each of them
 * requesting bus ownership when they need to.
 */

struct i3c_bus
{
  FAR struct i3c_dev_desc *cur_master;
  int id;
  uint32_t addrslots[((I3C_I2C_MAX_ADDR + 1) * 2) / I3C_BITS_PER_LONG];
  enum i3c_bus_mode mode;
  struct
  {
    unsigned long i3c;
    unsigned long i2c;
  } scl_rate;
  struct
  {
    struct list_node i3c;
    struct list_node i2c;
  } devs;
  mutex_t lock;
};

/* struct i3c_master_controller_ops - I3C master methods
 * @bus_init: hook responsible for the I3C bus initialization. You should
 *        at least call master_set_info() from there and set the bus mode.
 *        You can also put controller specific initialization in there.
 *        This method is mandatory.
 * @bus_cleanup: cleanup everything done in
 *       &i3c_master_controller_ops->bus_init().
 *       This method is optional.
 * @attach_i3c_dev: called every time an I3C device is attached to the bus.
 *        It can be after a DAA or when a device is statically declared
 *        by the FW, in which case it will only have a static address
 *        and the dynamic address will be 0.
 *        When this function is called, device information have not
 *        been retrieved yet.
 *        This is a good place to attach master controller specific
 *        data to I3C devices.
 *        This method is optional.
 * @reattach_i3c_dev: called every time an I3C device has its addressed
 *        changed. It can be because the device has been powered
 *        down and has lost its address, or it can happen when a
 *        device had a static address and has been assigned a
 *        dynamic address with SETDASA.
 *        This method is optional.
 * @detach_i3c_dev: called when an I3C device is detached from the bus.
 *        Usually happens when the master device is unregistered.
 *        This method is optional.
 * @do_daa: do a DAA (Dynamic Address Assignment) procedure. This is
 *        procedure should send an ENTDAA CCC command and then add all
 *        devices discovered sure the DAA using
 *        i3c_master_add_i3c_dev_locked().
 *        Add devices added with i3c_master_add_i3c_dev_locked() will then be
 *        attached or re-attached to the controller.
 *        This method is mandatory.
 * @supports_ccc_cmd: should return true if the CCC command is supported,
 *        false otherwise.
 *        This method is optional, if not provided the core assumes
 *        all CCC commands are supported.
 * @send_ccc_cmd: send a CCC command
 *        This method is mandatory.
 * @priv_xfers: do one or several private I3C SDR transfers
 *        This method is mandatory.
 * @attach_i2c_dev: called every time an I2C device is attached to the bus.
 * @detach_i2c_dev: called when an I2C device is detached from the bus.
 * @i2c_xfers: do one or several I2C transfers. Note that, unlike i3c
 *        transfers, the core does not guarantee that buffers attached to
 *        the transfers are DMA-safe. If drivers want to have DMA-safe
 *        buffers, they should use the i2c_get_dma_safe_msg_buf()
 *        and i2c_put_dma_safe_msg_buf() helpers provided by the I2C
 *        framework.
 *        This method is mandatory.
 * @request_ibi: attach an IBI handler to an I3C device. This implies
 *        defining an IBI handler and the constraints of the IBI (maximum
 *        payload length and number of pre-allocated slots).
 *        Some controllers support less IBI-capable devices than regular
 *        devices, so this method might return -%EBUSY if there's no
 *        more space for an extra IBI registration
 *        This method is optional.
 * @free_ibi: free an IBI previously requested with ->request_ibi(). The IBI
 *        should have been disabled with ->disable_irq() prior to that
 *        This method is mandatory only if ->request_ibi is not NULL.
 * @enable_ibi: enable the IBI. Only valid if ->request_ibi() has been called
 *        prior to ->enable_ibi(). The controller should first enable
 *        the IBI on the controller end (for example, unmask the hardware
 *        IRQ) and then send the ENEC CCC command (with the IBI flag set)
 *        to the I3C device.
 *        This method is mandatory only if ->request_ibi is not NULL.
 * @disable_ibi: disable an IBI. First send the DISEC CCC command with the
 *        IBI flag set and then deactivate the hardware IRQ on the
 *        controller end.
 *        This method is mandatory only if ->request_ibi is not NULL.
 * @recycle_ibi_slot: recycle an IBI slot. Called every time an IBI has been
 *        processed by its handler. The IBI slot should be put back
 *        in the IBI slot pool so that the controller can re-use it
 *        for a future IBI
 *        This method is mandatory only if ->request_ibi is not
 *        NULL.
 */

struct i3c_master_controller_ops
{
  CODE int (*bus_init)(FAR struct i3c_master_controller *master);
  CODE void (*bus_cleanup)(FAR struct i3c_master_controller *master);
  CODE int (*attach_i3c_dev)(FAR struct i3c_dev_desc *dev);
  CODE int (*reattach_i3c_dev)(FAR struct i3c_dev_desc *dev,
                               unsigned char old_dyn_addr);
  CODE void (*detach_i3c_dev)(FAR struct i3c_dev_desc *dev);
  CODE int (*do_daa)(FAR struct i3c_master_controller *master);
  CODE bool (*supports_ccc_cmd)(FAR struct i3c_master_controller *master,
                                FAR const struct i3c_ccc_cmd *cmd);
  CODE int (*send_ccc_cmd)(FAR struct i3c_master_controller *master,
                           FAR struct i3c_ccc_cmd *cmd);
  CODE int (*priv_xfers)(FAR struct i3c_dev_desc *dev,
                         FAR struct i3c_priv_xfer *xfers,
                         int nxfers);
  CODE int (*attach_i2c_dev)(FAR struct i3c_master_controller *master,
                             FAR const struct i2c_config_s *config);
  CODE int (*detach_i2c_dev)(FAR struct i3c_master_controller *master,
                             FAR const struct i2c_config_s *config);
  CODE int (*i2c_xfers)(FAR struct i3c_master_controller *master,
                        FAR const struct i2c_msg_s *xfers, int nxfers);
  CODE int (*request_ibi)(FAR struct i3c_dev_desc *dev,
                          FAR const struct i3c_ibi_setup *req);
  CODE void (*free_ibi)(FAR struct i3c_dev_desc *dev);
  CODE int (*enable_ibi)(FAR struct i3c_dev_desc *dev);
  CODE int (*disable_ibi)(FAR struct i3c_dev_desc *dev);
  CODE void (*recycle_ibi_slot)(FAR struct i3c_dev_desc *dev,
                                FAR struct i3c_ibi_slot *slot);
};

/* struct i3c_master_controller - I3C master controller object
 * @i3c_bus_id: the master id registered to the i3c bus
 * @i2c_bus_id: the i2c driver id registered to this master
 * @mode: mode selection by i2c device feature.
 * @max_i2c_scl_rate: max i2c scl rate in all i2c device
 * @ops: master operations. See &struct i3c_master_controller_ops
 * @this: an I3C device object representing this master. This device will
 *    be added to the list of I3C devs available on the bus
 * @i2c: I2C driver used for backward compatibility. This adapter is
 *   registered to the I2C subsystem to be as transparent as possible to
 *   existing I2C drivers

 * @secondary: true if the master is a secondary master
 * @init_done: true when the bus initialization is done
 * @bus: I3C bus exposed by this master
 *
 * A &struct i3c_master_controller has to be registered to the I3C subsystem
 * through i3c_master_register(). None of &struct i3c_master_controller
 * fields should be set manually, just pass appropriate values to
 * i3c_master_register().
 */

struct i3c_master_controller
{
  /* External implementation */

  int i3c_bus_id;                                  /* indicate a i3c bus id */
  int i2c_bus_id;                                  /* indicate a i2c bus id */
  enum i3c_bus_mode mode;                          /* select i3c bus mode by all device */
  unsigned long max_i2c_scl_rate;                  /* select max i2c scl rate by all i2c device */

  /* Internal implementation */

  FAR const struct i3c_master_controller_ops *ops; /* operation callback implemented in IP driver */
  FAR struct i3c_dev_desc *this;
  struct i2c_master_s i2c;
  unsigned int secondary : 1;
  unsigned int init_done : 1;
  struct i3c_bus bus;
};

/* struct i3c_generic_ibi_slot - an I3C generic IBI slot structure
 * @node: used to insert generic IBI slot list.
 * @base: an I3C IBI slot.
 *
 * This structure is struct i3c_ibi_slot list.
 */

struct i3c_generic_ibi_slot
{
  struct list_node node;
  struct i3c_ibi_slot base;
};

/* struct i3c_generic_ibi_pool - I3C master controller object
 * @lock: used to protect share resource.
 * @num_slots: the number of struct i3c_generic_ibi_slot.
 * @slots: the general slot used to manager payload infomation in pool.
 * @payload_buf: pending process messages from IBI handler.
 * @free_slots: a free slot list from a generic IBI pool.
 * @pending: a pending slots list from a generic IBI pool.
 *
 * This structure is used to manager all general ibi pool data,such as
 * pending slots or free slots.
 */

struct i3c_generic_ibi_pool
{
  spinlock_t lock;
  unsigned int num_slots;
  FAR struct i3c_generic_ibi_slot *slots;
  FAR void *payload_buf;
  struct list_node free_slots;
  struct list_node pending;
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i3c_dev_get_master_data
 *
 * Description:
 *   Get master private data attached to an I3C device descriptor.
 *
 *   This functions allows a master controller to attach per-device private
 *   data which can then be retrieved with i3c_dev_get_master_data().
 *
 * Input Parameters:
 *   dev - The I3C device descriptor to get private data from.
 *
 * Returned Value:
 *   The private data previously attached with i3c_dev_set_master_data() or
 *   NULL if no data has been attached to the device.
 ****************************************************************************/

static inline FAR void *
i3c_dev_get_master_data(FAR const struct i3c_dev_desc *dev)
{
  return dev->common.master_priv;
}

/****************************************************************************
 * Name: i3c_dev_set_master_data
 *
 * Description:
 *   Attach master private data to an I3C device descriptor.
 *
 *   This functions allows a master controller to attach per-device
 *   private data which can then be retrieved with i3c_dev_get_master_data().
 *
 * Input Parameters:
 *   dev  - The I3C device descriptor to attach private data to.
 *   data - Private data.
 ****************************************************************************/

static inline void
i3c_dev_set_master_data(FAR struct i3c_dev_desc *dev, FAR void *data)
{
  dev->common.master_priv = data;
}

/****************************************************************************
 * Name: i2c_dev_get_master_data
 *
 * Description:
 *   get Master private data attached to an I2C device descriptor.
 *
 * Input Parameters:
 *   dev - The I2C device descriptor to get private data from.
 *
 * Returned Value:
 *   The private data previously attached with i2c_dev_set_master_data() or
 *   NULL if no data has been attached to the device.
 ****************************************************************************/

static inline FAR void *
i2c_dev_get_master_data(FAR const struct i2c_dev_desc *dev)
{
  return dev->common.master_priv;
}

/****************************************************************************
 * Name: i2c_dev_set_master_data
 *
 * Description:
 *   Attach master private data to an I2C device descriptor.
 *
 * Input Parameters:
 *   dev  - The I2C device descriptor to attach private data to.
 *   data - Private data.
 ****************************************************************************/

static inline void
i2c_dev_set_master_data(FAR struct i2c_dev_desc *dev, FAR void *data)
{
  dev->common.master_priv = data;
}

/****************************************************************************
 * Name: i3c_dev_get_master
 *
 * Description:
 *   Get master used to communicate with a device
 *
 * Input Parameters:
 *   dev - I3C dev.
 *
 * Returned Value:
 *   The master controller driving @dev.
 *
 ****************************************************************************/

static inline FAR struct i3c_master_controller *
i3c_dev_get_master(FAR struct i3c_dev_desc *dev)
{
  return dev->common.master;
}

/****************************************************************************
 * Name: i2c_dev_get_master
 *
 * Description:
 *   Get master used to communicate with a device.
 *
 * Input Parameters:
 *   dev - I2C dev.
 *
 * Returned Value:
 *   The master controller driving dev.
 ****************************************************************************/

static inline FAR struct i3c_master_controller *
i2c_dev_get_master(FAR struct i2c_dev_desc *dev)
{
  return dev->common.master;
}

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: i3c_master_get_bus
 *
 * Description:
 *   Get the bus attached to a master.
 *
 *
 * Input Parameters:
 *   dev - Master object.
 *
 * Returned Value:
 *   The I3C bus @master is connected to.
 *
 ****************************************************************************/

static inline FAR struct i3c_bus *
i3c_master_get_bus(FAR struct i3c_master_controller *master)
{
  return &master->bus;
}

/****************************************************************************
 * Name: i3c_bus_normaluse_lock
 *
 * Description:
 *   I3C bus to take the lock on.
 *
 *   This function takes the bus lock for any operation that is not a
 *   maintenance operation (see i3c_bus_maintenance_lock() for
 *   non-exhaustive list of maintenance operations). Basically all
 *   communications with I3C devices are normal operations (HDR, SDR
 *   transfers or CCC commands that do not change bus state or I3C dynamic
 *   address).
 *
 * Input Parameters:
 *   bus - I3C bus to release the lock on.
 *
 ****************************************************************************/

void i3c_bus_normaluse_lock(FAR struct i3c_bus *bus);

/****************************************************************************
 * Name: i3c_bus_normaluse_unlock
 *
 * Description:
 *   Release the bus lock after a normal operation.
 *
 *   Should be called when a normal operation is done. See
 *   i3c_bus_normaluse_lock() for more details on what these
 *   normal operations are.
 *
 * Input Parameters:
 *   bus - I3C bus to release the lock on.
 *
 *
 ****************************************************************************/

void i3c_bus_normaluse_unlock(FAR struct i3c_bus *bus);

/****************************************************************************
 * Name: i3c_generic_ibi_alloc_pool
 *
 * Description:
 *   Create a generic IBI pool
 *
 *   Create a generic IBI pool based on the information provided in @req.
 *
 * Input Parameters:
 *   dev - The device this pool will be used for
 *   req - IBI setup request describing what the device driver expects
 *
 * Returned Value:
 *   A valid IBI pool in case of success, an ERR_PTR() otherwise.
 ****************************************************************************/

FAR struct i3c_generic_ibi_pool *
i3c_generic_ibi_alloc_pool(FAR struct i3c_dev_desc *dev,
                           FAR const struct i3c_ibi_setup *req);

/****************************************************************************
 * Name: i3c_generic_ibi_free_pool
 *
 * Description:
 *   Free a generic IBI pool
 *
 *   Free all IBI slots allated by a generic IBI pool.
 *
 * Input Parameters:
 *   pool - The IBI pool to free
 *
 ****************************************************************************/

void i3c_generic_ibi_free_pool(FAR struct i3c_generic_ibi_pool *pool);

/****************************************************************************
 * Name: i3c_generic_ibi_get_free_slot
 *
 * Description:
 *   Get a free slot from a generic IBI pool
 *
 *   Search for a free slot in a generic IBI pool.
 *   The slot should be returned to the pool using
 *   i3c_generic_ibi_recycle_slot
 *   when it's no longer needed.
 *
 * Input Parameters:
 *   pool - The pool to query an IBI slot on
 *
 * Returned Value:
 *   A pointer to a free slot, or NULL if there's no free slot available.
 *
 ****************************************************************************/

FAR struct i3c_ibi_slot *
i3c_generic_ibi_get_free_slot(FAR struct i3c_generic_ibi_pool *pool);

/****************************************************************************
 * Name: i3c_generic_ibi_recycle_slot
 *
 * Description:
 *   Return a slot to a generic IBI pool
 *
 *   Add an IBI slot back to its generic IBI pool. Should be called from the
 *   master driver struct_master_controller_ops->recycle_ibi() method.
 *
 * Input Parameters:
 *   pool - The pool to return the IBI slot to
 *   slot - IBI slot to recycle
 *
 ****************************************************************************/

void i3c_generic_ibi_recycle_slot(FAR struct i3c_generic_ibi_pool *pool,
                                  FAR struct i3c_ibi_slot *slot);

/****************************************************************************
 * Name: i3c_master_queue_ibi
 *
 * Description:
 *   Queue an IBI
 *
 *   Queue an IBI to the controller workqueue. The IBI handler attached
 *   to the dev will be called from a workqueue context.
 *
 * Input Parameters:
 *   dev  - The device this IBI is coming from
 *   slot - The IBI slot used to store the payload
 *
 ****************************************************************************/

void i3c_master_queue_ibi(FAR struct i3c_dev_desc *dev,
                          FAR struct i3c_ibi_slot *slot);

/****************************************************************************
 * Name: i3c_master_get_free_ibi_slot
 *
 * Description:
 *   Return a slot to a free IBI pool
 *
 * Input Parameters:
 *   dev - The device this IBI is coming from
 *
 ****************************************************************************/

FAR struct i3c_ibi_slot *i3c_master_get_free_ibi_slot(
                            FAR struct i3c_dev_desc *dev);

/****************************************************************************
 * Name: i3c_master_register
 *
 * Description:
 *   Register an I3C master.
 *
 *   This function takes care of everything for you:
 *     - Creates and initializes the I3C bus.
 *     - Registers all I3C charactor driver that supports I3C transfer.
 *     - Registers the I2C charactor driver that supports I2C transfer.
 *
 * Input Parameters:
 *   master - Master used to send frames on the bus.
 *   ops    - The master controller operations
 *   secondary - True if you are registering a secondary master. Will return
 *     -ENOTSUP if set to true since secondary masters are not yet supported
 *
 * return:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int i3c_master_register(FAR struct i3c_master_controller *master,
                        FAR const struct i3c_master_controller_ops *ops,
                        bool secondary);

/****************************************************************************
 * Name: i3c_master_unregister
 *
 * Description:
 *  Unregister an I3C master.
 *
 *  Basically undo everything done in i3c_master_register().
 *
 * Input Parameters:
 *   master - Master used to send frames on the bus.
 *
 ****************************************************************************/

void i3c_master_unregister(FAR struct i3c_master_controller *master);

/****************************************************************************
 * Name: i3c_master_disec_locked
 *
 * Description:
 *   Send a DISEC CCC command
 *
 *   Send a DISEC CCC command to disable some or all events coming from a
 *   Specific slave, or all devices if @addr is %I3C_BROADCAST_ADDR.
 *
 *   This function must be called with the bus lock held in write mode.
 *
 * Input Parameters:
 *   master - Master used to send frames on the bus.
 *   addr   - A valid I3C slave address or %I3C_BROADCAST_ADDR
 *   evts   - Events to disable
 *
 * Returned Value:
 *  0 in case of success, a positive I3C error code if the error is
 *  one of the official Mx error codes, and a negative error code otherwise
 *
 ****************************************************************************/

int i3c_master_disec_locked(FAR struct i3c_master_controller *master,
                            unsigned char addr, unsigned char evts);

/****************************************************************************
 * Name: i3c_master_enec_locked
 *
 * Description:
 *   Send an ENEC CCC command
 *
 *   Sends an ENEC CCC command to enable some or all events coming from a
 *   specific slave, or all devices if @addr is %I3C_BROADCAST_ADDR.
 *
 *   This function must be called with the bus lock held in write mode.
 *
 * Input Parameters:
 *   master - Master used to send frames on the bus.
 *   addr   - A valid I3C slave address or %I3C_BROADCAST_ADDR
 *   evts   - Events to enable
 *
 * Returned Value:
 *   0 in case of success, a positive I3C error code if the error is
 *   one of the official Mx error codes, and a negative error code otherwise.
 *
 ****************************************************************************/

int i3c_master_enec_locked(FAR struct i3c_master_controller *master,
                           unsigned char addr, unsigned char evts);

/****************************************************************************
 * Name: i3c_master_entdaa_locked
 *
 * Description:
 *   Start a DAA (Dynamic Address Assignment) procedure
 *
 *   This function must be called with the bus lock held in write mode.
 *
 *   Note that this function only sends the ENTDAA CCC command, all the logic
 *   behind dynamic address assignment has to be handled in the I3C master
 *   driver.
 *
 * Input Parameters:
 *   master - Master used to send frames on the bus.
 *
 * Returned Value:
 *  0 in case of success, a positive I3C error code if the error is
 *  one of the official Mx error codes, and a negative error code otherwise.
 *
 ****************************************************************************/

int i3c_master_entdaa_locked(FAR struct i3c_master_controller *master);

/****************************************************************************
 * Name: i3c_master_defslvs_locked
 *
 * Description:
 *   Send a DEFSLVS CCC command.
 *
 *   Send a DEFSLVS CCC command containing all the devices known to the
 *   master.
 *   This is useful when you have secondary masters on the bus to
 *   propagate device information.
 *
 *   This should be called after all I3C devices have been discovered (in
 *   other words, after the DAA procedure has finished) and instantiated in
 *   &i3c_master_controller_ops->bus_init().
 *   It should also be called if a master ACKed an Hot-Join request and
 *   assigneda dynamic address to the device joining the bus.
 *
 *   This function must be called with the bus lock held in write mode.
 *
 * Input Parameters:
 *   master - Master used to send frames on the bus.
 *
 * Returned Value:
 *   0 in case of success, a positive I3C error code if the error is
 *   one of the official Mx error codes, and a negative error code otherwise.
 *
 ****************************************************************************/

int i3c_master_defslvs_locked(FAR struct i3c_master_controller *master);

/****************************************************************************
 * Name: i3c_master_get_free_addr
 *
 * Description:
 *   Get a free address on the bus
 *
 *   This function must be called with the bus lock held in write mode.
 *
 * Input Parameters:
 *   master     - I3C master object.
 *   start_addr - Where to start searching.
 *
 * Returned Value:
 *   The first free address starting at @start_addr (included) or -ENOMEM
 *   if there's no more address available.
 *
 ****************************************************************************/

int i3c_master_get_free_addr(FAR struct i3c_master_controller *master,
                             unsigned char start_addr);

/****************************************************************************
 * Name: i3c_master_add_i3c_dev_locked
 *
 * Description:
 *   Add an I3C slave to the bus
 *
 *   This function is instantiating an I3C device object and adding it to
 *   the I3C device list. All device information are automatically retrieved
 *   using standard CCC commands.
 *
 *   The I3C device object is returned in case the master wants to attach
 *   private data to it using i3c_dev_set_master_data().
 *
 *   This function must be called with the bus lock held in write mode.
 *
 * Input Parameters:
 *   master - Master used to send frames on the bus
 *   addr   - I3C slave dynamic address assigned to the device
 *
 * Returned Value:
 *   0 in case of success, an negative error code otherwise.
 *
 ****************************************************************************/

int i3c_master_add_i3c_dev_locked(FAR struct i3c_master_controller *master,
                                  unsigned char addr);

/****************************************************************************
 * Name: i3c_master_do_daa
 *
 * Description:
 *   Do a DAA (Dynamic Address Assignment)
 *
 *   This function is instantiating an I3C device object and adding it to
 *   the I3C device list. All device information are automatically retrieved
 *   using standard CCC commands.
 *
 *   The I3C device object is returned in case the master wants to attach
 *   private data to it using i3c_dev_set_master_data().
 *
 *   This function must be called with the bus lock held in write mode.
 *
 * Input Parameters:
 *   master - Master doing the DAA
 *
 * Returned Value:
 *   0 in case of success, an negative error code otherwise.
 *
 ****************************************************************************/

int i3c_master_do_daa(FAR struct i3c_master_controller *master);

/****************************************************************************
 * Name: i3c_master_set_info
 *
 * Description:
 *   Set master device information
 *
 *   Set master device info. This should be called from
 *   &i3c_master_controller_ops->bus_init().
 *
 *   Not all &i3c_device_info fields are meaningful for a master device.
 *   Here is a list of fields that should be properly filled:
 *
 *   - &i3c_device_info->dyn_addr
 *   - &i3c_device_info->bcr
 *   - &i3c_device_info->dcr
 *   - &i3c_device_info->pid
 *   - &i3c_device_info->hdr_cap if %I3C_BCR_HDR_CAP bit is set in
 *    &i3c_device_info->bcr
 *
 *   This function must be called with the bus lock held in maintenance mode.
 *
 * Input Parameters:
 *   master - Master used to send frames on the bus
 *   info   -  I3C device information
 *
 * Returned Value:
 *   0 if @info contains valid information (not every piece of
 *   information can be checked, but we can at least make sure info->dyn_addr
 *   and @info->bcr are correct), -EINVAL otherwise.
 *
 ****************************************************************************/

int i3c_master_set_info(FAR struct i3c_master_controller *master,
                        FAR const struct i3c_device_info *info);

/****************************************************************************
 * Name: i3c_master_send_ccc_cmd_locked
 *
 * Description:
 *   Set master ccc command to i3c device of i3c bus
 *
 * Input Parameters:
 *   master -  Master used to send ccc frames on the bus
 *   cmd    -  Command data to send on the bus
 *
 * Returned Value:
 *   0 in case of success, an negative error code otherwise.
 ****************************************************************************/

int i3c_master_send_ccc_cmd_locked(
              FAR struct i3c_master_controller *master,
              FAR struct i3c_ccc_cmd *cmd);

#endif /* __INCLUDE_NUTTX_I3C_MASTER_H */

/****************************************************************************
 * include/nuttx/analog/dac.h
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

#ifndef __INCLUDE_NUTTX_ANALOG_DAC_H
#define __INCLUDE_NUTTX_ANALOG_DAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default configuration settings that may be overridden in the board
 * configuration file.  The configured size is limited to 255 to fit into a
 * uint8_t.
 */

#if !defined(CONFIG_DAC_FIFOSIZE)
#  define CONFIG_DAC_FIFOSIZE 8
#elif CONFIG_DAC_FIFOSIZE > 255
#  undef  CONFIG_DAC_FIFOSIZE
#  define CONFIG_DAC_FIFOSIZE 255
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct dac_msg_s
{
  uint8_t      am_channel;               /* The 8-bit DAC Channel */
  int32_t      am_data;                  /* DAC convert result (4 bytes) */
} end_packed_struct;

struct dac_fifo_s
{
  sem_t         af_sem;                  /* Counting semaphore */
  uint8_t       af_head;                 /* Index to the head [IN] index
                                          * in the circular buffer */
  uint8_t       af_tail;                 /* Index to the tail [OUT] index
                                          * in the circular buffer */
                                         /* Circular buffer of DAC messages */
  struct dac_msg_s af_buffer[CONFIG_DAC_FIFOSIZE];
};

/* This structure defines all of the operations provided by the architecture
 * specific logic.  All fields must be provided with non-NULL function
 * pointers by the caller of dac_register().
 */

struct dac_dev_s;
struct dac_ops_s
{
  /* Reset the DAC device.  Called early to initialize the hardware. This
   * is called, before ao_setup() and on error conditions.
   */

  CODE void (*ao_reset)(FAR struct dac_dev_s *dev);

  /* Configure the DAC. This method is called the first time that the DAC
   * device is opened.  This will occur when the port is first opened.
   * This setup includes configuring and attaching DAC interrupts.
   * Interrupts are all disabled upon return.
   */

  CODE int (*ao_setup)(FAR struct dac_dev_s *dev);

  /* Disable the DAC.  This method is called when the DAC device is closed.
   * This method reverses the operation of the setup method.
   */

  CODE void (*ao_shutdown)(FAR struct dac_dev_s *dev);

  /* Call to enable or disable TX interrupts */

  CODE void (*ao_txint)(FAR struct dac_dev_s *dev, bool enable);

  /* This method will send one message on the DAC */

  CODE int (*ao_send)(FAR struct dac_dev_s *dev, FAR struct dac_msg_s *msg);

  /* All ioctl calls will be routed through this method */

  CODE int (*ao_ioctl)(FAR struct dac_dev_s *dev, int cmd,
                       unsigned long arg);
};

/* This is the device structure used by the driver.  The caller of
 * dac_register() must allocate and initialize this structure.  The
 * calling logic needs to set all fields to zero except:
 *
 *   The elements of 'ad_ops', and 'ad_priv'
 *
 * The common logic will initialize all semaphores.
 */

struct dac_dev_s
{
  const struct dac_ops_s *ad_ops;      /* Arch-specific operations */
  void                   *ad_priv;     /* Used by the arch-specific logic */
  uint8_t                 ad_ocount;   /* The number of times the device has
                                        * been opened */
  uint8_t                 ad_nchannel; /* Number of dac channel */
  sem_t                   ad_closesem; /* Locks out new opens while close is
                                        * in progress */
  struct dac_fifo_s       ad_xmit;     /* Describes receive FIFO */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Name: dac_register
 *
 * Description:
 *   Register a dac driver.
 *
 * Input Parameters:
 *    path - The full path to the DAC device to be registered.  This could
 *           be, as an example, "/dev/dac0"
 *    dev - An instance of the device-specific DAC interface
 *
 * Returned Value:
 *    Zero on success; A negated errno value on failure.
 *
 ****************************************************************************/

int dac_register(FAR const char *path, FAR struct dac_dev_s *dev);

/****************************************************************************
 * Name: dac_txdone
 *
 * Description:
 *   Called from the DAC interrupt handler at the completion of a send
 *    operation.
 *
 * Input Parameters:
 *    dev - An instance of the device-specific DAC interface
 *
 * Returned Value:
 *   OK on success; a negated errno on failure.
 *
 ****************************************************************************/

int dac_txdone(FAR struct dac_dev_s *dev);

/****************************************************************************
 * DAC Initialization functions
 *
 * Architecture-specific versions will have prototypes in architect-specific
 * header files.  Common DAC implementations in drivers/analog will have
 * prototypes listed below.
 *
 ****************************************************************************/

FAR struct dac_dev_s *up_ad5410initialize(FAR struct spi_dev_s *spi,
                                          unsigned int devno);

FAR struct dac_dev_s *dac7571_initialize(FAR struct i2c_master_s *i2c,
                                         uint8_t addr);

FAR struct dac_dev_s *dac7554_initialize(FAR struct spi_dev_s *spi,
                                         int spidev);

/****************************************************************************
 * Name: lmp92001_dac_initialize
 *
 * Description:
 *   Initialize DAC
 *
 * Input Parameters:
 *   I2C Port number
 *   Device address
 *
 * Returned Value:
 *   Valid LM92001 device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct dac_dev_s *lmp92001_dac_initialize(FAR struct i2c_master_s *i2c,
                                              uint8_t addr);

/****************************************************************************
 * Name: mcp48xx_initialize
 *
 * Description:
 *   Initialize DAC
 *
 * Input Parameters:
 *    spi - SPI driver instance
 *    spidev - SPI Chip Select number
 *
 * Returned Value:
 *   Valid MCP48XX device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct dac_dev_s *mcp48xx_initialize(FAR struct spi_dev_s *spi,
                                         uint32_t spidev);

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_ANALOG_DAC_H */

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_I2C_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include "chip/kinetis_i2c.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_i2cbus_initialize
 *
 * Description:
 *   Initialize the selected I2C port. And return a unique instance of struct
 *   struct i2c_master_s.  This function may be called to obtain multiple
 *   instances of the interface, each of which may be set up with a
 *   different frequency and slave address.
 *
 * Input Parameter:
 *   Port number (for hardware that has multiple I2C interfaces)
 *
 * Returned Value:
 *   Valid I2C device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct i2c_master_s *kinetis_i2cbus_initialize(int port);

/****************************************************************************
 * Name: kinetis_i2cbus_uninitialize
 *
 * Description:
 *   De-initialize the selected I2C port, and power down the device.
 *
 * Input Parameter:
 *   Device structure as returned by the lpc43_i2cbus_initialize()
 *
 * Returned Value:
 *   OK on success, ERROR when internal reference count mismatch or dev
 *   points to invalid hardware device.
 *
 ****************************************************************************/

int kinetis_i2cbus_uninitialize(FAR struct i2c_master_s *dev);

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_I2C_H */

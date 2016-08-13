/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "kinetis.h"
#include "kinetis_i2c.h"
#include "teensy-3x.h"

#if defined(CONFIG_KINETIS_I2C0) || defined(CONFIG_KINETIS_I2C1)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: kinetis_i2cdev_initialize
 *
 * Description:
 *   Called to configure I2C
 *
 ************************************************************************************/

void kinetis_i2cdev_initialize(void)
{
  i2c_dev = NULL;

#if defined(CONFIG_KINETIS_I2C0)
  i2c_dev = kinetis_i2cbus_initialize(0);
#if defined(CONFIG_I2C_DRIVER)
  i2c_register(i2c_dev, 0);
#endif
#endif

#if defined(CONFIG_KINETIS_I2C1)
#error Not yet supported in kinetis driver
  i2c_dev  = kinetis_i2cbus_initialize(1);
#if defined(CONFIG_I2C_DRIVER)
  i2c_register(i2c_dev, 1);
#endif
#endif
}

#endif /* CONFIG_KINETIS_I2C0 || CONFIG_KINETIS_I2C1 */

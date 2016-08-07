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
#include "teensy-3x.h"
#include "kinetis_i2c.h"

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

void weak_function kinetis_i2cdev_initialize(void)
{
#if defined(CONFIG_KINETIS_I2C0)
  kinetis_i2cbus_initialize(0);
#endif

#if defined(CONFIG_KINETIS_I2C1)
  kinetis_i2cbus_initialize(1);
#endif
}


#endif /* CONFIG_KINETIS_I2C0 || CONFIG_KINETIS_I2C1 */

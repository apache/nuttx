/****************************************************************************
 * boards/arm/stm32h7/nucleo-h753zi/src/stm32_ssd1306.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * About this driver
 ****************************************************************************/

/* This is the board-level driver for the SSD1306 OLED display.
 *
 * WHAT IS SSD1306?
 * ----------------
 * The SSD1306 is a small monochrome OLED display controller. Common sizes
 * are 128x64 or 128x32 pixels. It communicates via I2C or SPI.
 * This driver uses I2C.
 *
 * DRIVER ARCHITECTURE IN NUTTX:
 * -----------------------------
 * NuttX uses a layered driver architecture:
 *
 *   +---------------------+
 *   |   Application       |  <- Your app opens /dev/lcd0
 *   +---------------------+
 *            |
 *   +---------------------+
 *   |   LCD Dev Layer     |  <- lcddev_register() creates /dev/lcdX
 *   +---------------------+
 *            |
 *   +---------------------+
 *   |   SSD1306 Driver    |  <- ssd1306_initialize() from NuttX
 *   +---------------------+
 *            |
 *   +---------------------+
 *   |   I2C Master        |  <- stm32_i2cbus_initialize() from arch
 *   +---------------------+
 *            |
 *   +---------------------+
 *   |   Hardware (I2C2)   |  <- Physical pins PF0/PF1
 *   +---------------------+
 *
 * THIS FILE'S ROLE:
 * -----------------
 * This file is the "glue" that connects the NuttX SSD1306 driver to our
 * specific board (Nucleo-H753ZI). It:
 *   1. Gets the I2C bus interface
 *   2. Passes it to the SSD1306 driver
 *   3. Provides helper functions for power control
 *
 * IMPORTANT: The lcddev_register() call must be made from stm32_bringup.c
 * AFTER board_lcd_getdev() returns, NOT inside board_lcd_getdev().
 * This is because lcddev_register() internally calls board_lcd_getdev(),
 * which would create infinite recursion.
 *
 * WHY /dev/lcdX AND NOT /dev/fbX?
 * -------------------------------
 * - /dev/fbX (framebuffer): Used for displays that need a full memory
 *   buffer in RAM. Good for large color displays (like ST7796 480x320).
 *
 * - /dev/lcdX (LCD interface): Used for smaller displays with internal
 *   memory. The SSD1306 has 1KB RAM inside the chip itself, so we don't
 *   need a big framebuffer in our MCU's RAM.
 *
 * BRIGHTNESS CONTROL:
 * -------------------
 * The SSD1306 requires 3 registers to effectively control brightness:
 * - 0x81: Contrast (0-255)
 * - 0xD9: Pre-charge period
 * - 0xDB: VCOMH deselect level
 *
 * The NuttX driver only sets contrast (0x81), which has minimal visual
 * effect. This driver sends all 3 commands directly via I2C for real
 * brightness control.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Standard C headers */

#include <errno.h>                    /* Error codes like ENODEV, EINVAL */
#include <syslog.h>                   /* For logging messages */
#include <debug.h>                    /* Debug macros */

/* NuttX system headers */

#include <nuttx/board.h>              /* Board-level interfaces */

/* LCD-related headers */

#include <nuttx/lcd/lcd.h>            /* Generic LCD interface */
#include <nuttx/lcd/ssd1306.h>        /* SSD1306-specific driver */

/* I2C header */

#include <nuttx/i2c/i2c_master.h>     /* I2C master interface */

/* Our board-specific header with pin definitions and prototypes */

#include "nucleo-h753zi.h"

/****************************************************************************
 * Conditional Compilation
 ****************************************************************************/

/* This entire file is only compiled if BOTH conditions are true:
 *
 * 1. CONFIG_LCD_SSD1306: The NuttX SSD1306 driver is enabled
 *    (menuconfig: Device Drivers -> LCD -> SSD1306)
 *
 * 2. CONFIG_NUCLEO_H753ZI_SSD1306_ENABLE: Our board-specific option
 *    (menuconfig: Board Selection -> I2C Devices -> SSD1306)
 *
 * If either is missing, this file compiles to nothing (empty).
 */

#if defined(CONFIG_LCD_SSD1306) && \
    defined(CONFIG_NUCLEO_H753ZI_SSD1306_ENABLE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* These macros translate the Kconfig values (accessed via nucleo-h753zi.h)
 * to simpler names used in this file.
 *
 * WHY THIS INDIRECTION?
 * The Kconfig values have long names like NUCLEO_SSD1306_I2C_BUS.
 * Using shorter names makes the code more readable.
 * Also, if we ever need to change the source of these values,
 * we only change it here.
 */

#define SSD1306_I2C_BUS       NUCLEO_SSD1306_I2C_BUS       /* Which I2C: 1-4 */
#define SSD1306_I2C_ADDR      NUCLEO_SSD1306_I2C_ADDR      /* 0x3C or 0x3D */
#define SSD1306_I2C_FREQUENCY NUCLEO_SSD1306_I2C_FREQUENCY /* 100k or 400k */
#define SSD1306_POWER_PERCENT NUCLEO_SSD1306_POWER_PERCENT /* 0-100% */
#define SSD1306_DEVPATH       NUCLEO_SSD1306_DEVPATH       /* "/dev/lcd0" */
#define SSD1306_DEVNO         NUCLEO_SSD1306_DEVNO         /* 0 */
#define SSD1306_DEVNAME       NUCLEO_SSD1306_DEVNAME       /* "ssd1306" */

/* SSD1306 Command Bytes */

#define SSD1306_CMD_BYTE          0x00  /* Control byte: Co=0, D/C#=0 */
#define SSD1306_CMD_SETCONTRAST   0x81  /* Set contrast control */
#define SSD1306_CMD_SETPRECHARGE  0xd9  /* Set pre-charge period */
#define SSD1306_CMD_SETVCOMDETECT 0xdb  /* Set VCOMH deselect level */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These are module-level variables (static = only visible in this file).
 *
 * g_ssd1306_i2c: Pointer to the I2C master interface. We get this from
 *                our board's I2C driver and pass it to the SSD1306 driver.
 *
 * g_ssd1306_lcd: Pointer to the LCD device structure. The SSD1306 driver
 *                returns this, and we use it to control the display.
 *
 * Both start as NULL and are set during initialization.
 */

static struct i2c_master_s *g_ssd1306_i2c = NULL;
static struct lcd_dev_s    *g_ssd1306_lcd = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ssd1306_send_cmd_arg
 *
 * Description:
 *   Send a command with one argument byte to the SSD1306 via I2C.
 *
 * Input Parameters:
 *   cmd - Command byte
 *   arg - Argument byte
 *
 * Returned Value:
 *   OK on success, negative errno on failure
 *
 ****************************************************************************/

static int ssd1306_send_cmd_arg(uint8_t cmd, uint8_t arg)
{
  struct i2c_msg_s msg;
  uint8_t buf[3];
  int ret;

  if (!g_ssd1306_i2c)
    {
      return -ENODEV;
    }

  /* SSD1306 I2C protocol for double-byte commands:
   * [Control byte: 0x00] [Command] [Argument]
   *
   * Control byte 0x00 = Co=0 (continuous), D/C#=0 (command mode)
   * This tells SSD1306 that following bytes are all commands.
   */

  buf[0] = SSD1306_CMD_BYTE;  /* Control byte: command mode */
  buf[1] = cmd;
  buf[2] = arg;

  msg.frequency = SSD1306_I2C_FREQUENCY;
  msg.addr      = SSD1306_I2C_ADDR;
  msg.flags     = 0;          /* Write */
  msg.buffer    = buf;
  msg.length    = 3;

  ret = I2C_TRANSFER(g_ssd1306_i2c, &msg, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "SSD1306: I2C cmd 0x%02x arg 0x%02x failed: %d\n",
             cmd, arg, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ssd1306_set_brightness_raw
 *
 * Description:
 *   Set display brightness using all 3 required SSD1306 registers.
 *   This bypasses the NuttX driver to provide real brightness control.
 *
 *   The SSD1306 brightness is controlled by:
 *   - Contrast register (0x81): 0-255, controls OLED segment current
 *   - Pre-charge period (0xD9): affects charge pump timing
 *   - VCOMH level (0xDB): affects voltage levels
 *
 *   For effective dimming, all 3 must be adjusted together.
 *
 * Input Parameters:
 *   percent - Brightness 0-100%
 *
 * Returned Value:
 *   OK on success, negative errno on failure
 *
 ****************************************************************************/

static int ssd1306_set_brightness_raw(int percent)
{
  uint8_t contrast;
  uint8_t precharge;
  uint8_t vcomh;
  int ret;

  /* Clamp to valid range */

  if (percent < 0)
    {
      percent = 0;
    }
  else if (percent > 100)
    {
      percent = 100;
    }

  /* Calculate register values based on percentage.
   *
   * Contrast: Linear scale 0-255
   *
   * Pre-charge (0xD9): Controls charge/discharge time
   *   - Low nibble: phase 1 (1-15 DCLKs)
   *   - High nibble: phase 2 (1-15 DCLKs)
   *   - Range: 0x11 (dimmest) to 0xFF (brightest)
   *
   * VCOMH (0xDB): Deselect voltage level
   *   - 0x00 = 0.65 x VCC (dimmest)
   *   - 0x20 = 0.77 x VCC
   *   - 0x30 = 0.83 x VCC
   *   - 0x40 = 1.00 x VCC (brightest, may cause damage on some displays)
   *
   * Using linear interpolation for smoother transitions.
   */

  /* Contrast: direct linear mapping */

  contrast = (uint8_t)((percent * 255) / 100);

  /* Pre-charge: scale from 0x11 to 0xF1
   * phase1 = 1 + (percent * 14 / 100)  -> 1 to 15
   * phase2 = 1 + (percent * 14 / 100)  -> 1 to 15
   */

    {
    uint8_t phase1;
    uint8_t phase2;

    phase1 = 1 + (uint8_t)((percent * 14) / 100);
    phase2 = 1 + (uint8_t)((percent * 14) / 100);
    precharge = (phase2 << 4) | phase1;
    }

  /* VCOMH: scale from 0x00 to 0x30 (avoid 0x40 for safety)
   * Linear: 0x00 at 0%, 0x30 at 100%
   */

  vcomh = (uint8_t)((percent * 0x30) / 100);

  syslog(LOG_INFO,
         "SSD1306: Brightness %d%% -> con=0x%02x pre=0x%02x vcom=0x%02x\n",
         percent, contrast, precharge, vcomh);

  /* Send all three commands */

  ret = ssd1306_send_cmd_arg(SSD1306_CMD_SETCONTRAST, contrast);
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_send_cmd_arg(SSD1306_CMD_SETPRECHARGE, precharge);
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_send_cmd_arg(SSD1306_CMD_SETVCOMDETECT, vcomh);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: calculate_power_level
 *
 * Description:
 *   Convert a percentage (0-100) to an SSD1306 power level.
 *
 *   The SSD1306 driver only understands two power states:
 *   - 0 = display OFF (sleep mode, low power)
 *   - 1 = display ON (normal operation)
 *
 *   So any percentage > 0 means "turn on", and 0 means "turn off".
 *   Brightness/contrast is a separate setting handled by the driver.
 *
 * Input Parameters:
 *   percent - Power level as percentage (0-100)
 *
 * Returned Value:
 *   0 = power off, 1 = power on
 *
 ****************************************************************************/

static int calculate_power_level(int percent)
{
  if (percent <= 0)
    {
      return 0;  /* Power off */
    }
  else
    {
      return 1;  /* Power on */
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD hardware. This is called early during boot from
 *   stm32_bringup.c in the display initialization phase.
 *
 *   This function does NOT initialize the display itself - it only
 *   prepares the I2C interface. The actual display initialization
 *   happens in board_lcd_getdev().
 *
 * Why split into two functions?
 *   This follows the NuttX LCD driver pattern:
 *   - board_lcd_initialize(): Prepare the bus (I2C/SPI)
 *   - board_lcd_getdev(): Initialize and return the LCD device
 *
 * Returned Value:
 *   OK (0) on success
 *   Negative errno on failure (e.g., -ENODEV if I2C bus not available)
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  int ret;

  /* Log what we're doing - helps with debugging */

  lcdinfo("SSD1306: Initializing OLED display\n");
  lcdinfo("SSD1306:   I2C Bus: I2C%d\n", SSD1306_I2C_BUS);
  lcdinfo("SSD1306:   Address: 0x%02X\n", SSD1306_I2C_ADDR);
  lcdinfo("SSD1306:   Frequency: %lu Hz\n",
         (unsigned long)SSD1306_I2C_FREQUENCY);
  lcdinfo("SSD1306:   Brightness: %d%%\n", SSD1306_POWER_PERCENT);

  /* Step 1: Register I2C device for tracking and debugging
   *
   * This is OPTIONAL but useful. Our stm32_i2c.c driver keeps a list
   * of all I2C devices on each bus. This helps when debugging to see
   * what devices are registered.
   *
   * If this fails, we continue anyway - it's not critical.
   */

  ret = stm32_i2c_register_device(SSD1306_I2C_BUS,
                                  SSD1306_I2C_ADDR,
                                  SSD1306_I2C_FREQUENCY,
                                  SSD1306_DEVNAME);
  if (ret < 0)
    {
      syslog(LOG_WARNING,
             "SSD1306: WARNING - Failed to register I2C device: %d\n", ret);

      /* Continue anyway - registration is optional */
    }

  /* Step 2: Get the I2C master interface
   *
   * stm32_i2c_get_master() returns a pointer to the I2C bus structure.
   * This structure contains function pointers for I2C operations
   * (transfer, reset, etc.) that the SSD1306 driver will use.
   *
   * If this fails, we cannot continue - we need I2C to talk to display.
   */

  g_ssd1306_i2c = stm32_i2c_get_master(SSD1306_I2C_BUS);
  if (!g_ssd1306_i2c)
    {
      syslog(LOG_ERR, "SSD1306: ERROR - Failed to get I2C%d master\n",
             SSD1306_I2C_BUS);

      /* Cleanup: unregister the device we registered in step 1 */

      stm32_i2c_unregister_device(SSD1306_I2C_BUS, SSD1306_I2C_ADDR);
      return -ENODEV;
    }

  lcdinfo("SSD1306: I2C interface ready\n");
  return OK;
}

/****************************************************************************
 * Name: board_lcd_getdev
 *
 * Description:
 *   Get the LCD device structure and initialize the display.
 *
 *   This function:
 *   1. Calls ssd1306_initialize() to init the display hardware
 *   2. Turns on the display if configured to do so
 *   3. Sets the brightness level from Kconfig (using direct I2C)
 *
 *   IMPORTANT: Do NOT call lcddev_register() from inside this function!
 *   lcddev_register() internally calls board_lcd_getdev(), which would
 *   create infinite recursion. The lcddev_register() call must be made
 *   from stm32_bringup.c AFTER this function returns.
 *
 * Input Parameters:
 *   devno - Device number (0 for first LCD, 1 for second, etc.)
 *           This becomes the X in /dev/lcdX
 *
 * Returned Value:
 *   Pointer to lcd_dev_s structure on success
 *   NULL on failure
 *
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int devno)
{
  int power_level;

  /* Safety check: make sure board_lcd_initialize() was called first */

  if (!g_ssd1306_i2c)
    {
      syslog(LOG_ERR,
             "SSD1306: ERROR - I2C not initialized. "
             "Call board_lcd_initialize() first\n");
      return NULL;
    }

  /* If already initialized, just return the existing device.
   * This prevents re-initialization if called multiple times.
   */

  if (g_ssd1306_lcd != NULL)
    {
      return g_ssd1306_lcd;
    }

  /* Step 1: Initialize the SSD1306 driver
   *
   * ssd1306_initialize() is provided by NuttX (drivers/lcd/ssd1306.c).
   * It:
   *   - Sends initialization commands to the display via I2C
   *   - Allocates internal structures
   *   - Returns a lcd_dev_s pointer we can use to control the display
   *
   * Parameters:
   *   g_ssd1306_i2c: The I2C interface to use
   *   NULL: No SPI interface (we're using I2C)
   *   devno: Device number for this LCD
   */

  lcdinfo("SSD1306: Binding to I2C%d (device %d)\n",
         SSD1306_I2C_BUS, devno);

  g_ssd1306_lcd = ssd1306_initialize(g_ssd1306_i2c, NULL, devno);
  if (!g_ssd1306_lcd)
    {
      syslog(LOG_ERR,
             "SSD1306: ERROR - ssd1306_initialize() failed\n");
      return NULL;
    }

  lcdinfo("SSD1306: Driver initialized successfully\n");

  /* Step 2: Turn on the display (if configured)
   *
   * The power level comes from Kconfig (SSD1306_POWER_PERCENT).
   * - 0%: Keep display off
   * - 1-100%: Turn display on
   *
   * setpower() is a function pointer in the lcd_dev_s structure.
   * The SSD1306 driver implements it to send the "Display ON" command.
   */

  power_level = calculate_power_level(SSD1306_POWER_PERCENT);

  if (power_level > 0)
    {
      lcdinfo("SSD1306: Turning on display\n");
      g_ssd1306_lcd->setpower(g_ssd1306_lcd, power_level);

      /* Step 3: Set display brightness using direct I2C commands
       *
       * The NuttX setcontrast() only sends the contrast register (0x81),
       * which has minimal visual effect on brightness. For real brightness
       * control, we need to also adjust pre-charge and VCOMH registers.
       */

      ssd1306_set_brightness_raw(SSD1306_POWER_PERCENT);
    }
  else
    {
      lcdinfo("SSD1306: Display is OFF (power: 0%%)\n");
    }

  /* NOTE: Do NOT call lcddev_register() here!
   *
   * lcddev_register() internally calls board_lcd_getdev() to get
   * the LCD device. If we call it here, we create infinite recursion:
   *
   *   board_lcd_getdev() -> lcddev_register() -> board_lcd_getdev() -> ...
   *
   * The lcddev_register() call must be made from stm32_bringup.c
   * AFTER this function returns successfully.
   */

  return g_ssd1306_lcd;
}

/****************************************************************************
 * Name: board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the LCD and release resources.
 *
 *   This should be called when the system is shutting down or if you
 *   need to reconfigure the LCD. It:
 *   - Turns off the display (saves power)
 *   - Unregisters the I2C device from our tracking system
 *   - Clears the global pointers
 *
 *   Note: We don't uninitialize the I2C bus itself because other
 *   devices might be using it (sensors, etc.).
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  lcdinfo("SSD1306: Uninitializing display\n");

  /* Power off display to save energy */

  if (g_ssd1306_lcd)
    {
      g_ssd1306_lcd->setpower(g_ssd1306_lcd, 0);
      g_ssd1306_lcd = NULL;
    }

  /* Unregister from our I2C device tracking system */

  stm32_i2c_unregister_device(SSD1306_I2C_BUS, SSD1306_I2C_ADDR);

  /* Clear the I2C pointer but don't uninitialize the bus
   * because other devices might be using it.
   */

  g_ssd1306_i2c = NULL;

  lcdinfo("SSD1306: Uninitialized\n");
}

/****************************************************************************
 * Name: stm32_ssd1306_get_devpath
 *
 * Description:
 *   Get the configured device path for the SSD1306.
 *
 *   Useful when an application needs to know which /dev/lcdX to open.
 *   Instead of hardcoding "/dev/lcd0", the app can call this function.
 *
 * Example usage in an application:
 *   const char *path = stm32_ssd1306_get_devpath();
 *   int fd = open(path, O_RDWR);
 *
 * Returned Value:
 *   Pointer to device path string (e.g., "/dev/lcd0")
 *
 ****************************************************************************/

const char *stm32_ssd1306_get_devpath(void)
{
  return SSD1306_DEVPATH;
}

/****************************************************************************
 * Name: stm32_ssd1306_set_power
 *
 * Description:
 *   Change SSD1306 display power at runtime.
 *
 *   This allows turning the display on/off after initialization.
 *   Useful for:
 *   - Power saving (turn off when not needed)
 *   - Screen blanking
 *   - User-controlled display toggle
 *
 * Input Parameters:
 *   percent - Power level 0-100%
 *             0 = off, 1-100 = on (and sets brightness)
 *
 * Returned Value:
 *   OK on success
 *   -ENODEV if LCD not initialized
 *   -EINVAL if percentage is invalid
 *
 ****************************************************************************/

int stm32_ssd1306_set_power(int percent)
{
  int power_level;

  /* Check if the LCD was initialized */

  if (!g_ssd1306_lcd)
    {
      syslog(LOG_ERR, "SSD1306: ERROR - Not initialized\n");
      return -ENODEV;
    }

  /* Validate input range */

  if (percent < 0 || percent > 100)
    {
      syslog(LOG_ERR,
             "SSD1306: ERROR - Invalid power: %d (must be 0-100)\n",
             percent);
      return -EINVAL;
    }

  /* Convert percentage to power level and apply */

  power_level = calculate_power_level(percent);

  lcdinfo("SSD1306: Setting power to %d%%\n", percent);

  g_ssd1306_lcd->setpower(g_ssd1306_lcd, power_level);

  /* If turning on, also set brightness */

  if (power_level > 0)
    {
      ssd1306_set_brightness_raw(percent);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_ssd1306_set_brightness
 *
 * Description:
 *   Change SSD1306 display brightness at runtime using direct I2C.
 *
 *   This function provides real brightness control by adjusting all 3
 *   required SSD1306 registers (contrast, pre-charge, VCOMH).
 *
 * Input Parameters:
 *   percent - Brightness level 0-100%
 *
 * Returned Value:
 *   OK on success
 *   -ENODEV if LCD not initialized
 *   -EINVAL if percentage is invalid
 *
 ****************************************************************************/

int stm32_ssd1306_set_brightness(int percent)
{
  /* Check if the I2C interface was initialized */

  if (!g_ssd1306_i2c)
    {
      syslog(LOG_ERR, "SSD1306: ERROR - Not initialized\n");
      return -ENODEV;
    }

  /* Validate input range */

  if (percent < 0 || percent > 100)
    {
      syslog(LOG_ERR,
             "SSD1306: ERROR - Invalid brightness: %d (must be 0-100)\n",
             percent);
      return -EINVAL;
    }

  return ssd1306_set_brightness_raw(percent);
}

#endif /* CONFIG_LCD_SSD1306 && CONFIG_NUCLEO_H753ZI_SSD1306_ENABLE */

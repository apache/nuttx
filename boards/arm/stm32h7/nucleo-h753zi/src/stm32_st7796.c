/****************************************************************************
 * boards/arm/stm32h7/nucleo-h753zi/src/stm32_st7796.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <syslog.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/video/fb.h>
#include <nuttx/lcd/st7796.h>
#include <nuttx/clock.h>
#include <nuttx/signal.h>

#include "nucleo-h753zi.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"

#if defined(CONFIG_LCD_ST7796) && defined(CONFIG_NUCLEO_H753ZI_ST7796_ENABLE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI port selection from Kconfig */

#ifdef CONFIG_NUCLEO_H753ZI_ST7796_SPI1
#  define ST7796_SPI_PORTNO 1
#  ifndef CONFIG_STM32H7_SPI1
#    error "ST7796 configured for SPI1 but CONFIG_STM32H7_SPI1 not enabled"
#  endif
#  ifndef CONFIG_NUCLEO_H753ZI_SPI1_ENABLE
#    error "ST7796 configured for SPI1 but board SPI1 not enabled"
#  endif
#elif defined(CONFIG_NUCLEO_H753ZI_ST7796_SPI2)
#  define ST7796_SPI_PORTNO 2
#  ifndef CONFIG_STM32H7_SPI2
#    error "ST7796 configured for SPI2 but CONFIG_STM32H7_SPI2 not enabled"
#  endif
#  ifndef CONFIG_NUCLEO_H753ZI_SPI2_ENABLE
#    error "ST7796 configured for SPI2 but board SPI2 not enabled"
#  endif
#elif defined(CONFIG_NUCLEO_H753ZI_ST7796_SPI3)
#  define ST7796_SPI_PORTNO 3
#  ifndef CONFIG_STM32H7_SPI3
#    error "ST7796 configured for SPI3 but CONFIG_STM32H7_SPI3 not enabled"
#  endif
#  ifndef CONFIG_NUCLEO_H753ZI_SPI3_ENABLE
#    error "ST7796 configured for SPI3 but board SPI3 not enabled"
#  endif
#elif defined(CONFIG_NUCLEO_H753ZI_ST7796_SPI4)
#  define ST7796_SPI_PORTNO 4
#  ifndef CONFIG_STM32H7_SPI4
#    error "ST7796 configured for SPI4 but CONFIG_STM32H7_SPI4 not enabled"
#  endif
#  ifndef CONFIG_NUCLEO_H753ZI_SPI4_ENABLE
#    error "ST7796 configured for SPI4 but board SPI4 not enabled"
#  endif
#elif defined(CONFIG_NUCLEO_H753ZI_ST7796_SPI5)
#  define ST7796_SPI_PORTNO 5
#  ifndef CONFIG_STM32H7_SPI5
#    error "ST7796 configured for SPI5 but CONFIG_STM32H7_SPI5 not enabled"
#  endif
#  ifndef CONFIG_NUCLEO_H753ZI_SPI5_ENABLE
#    error "ST7796 configured for SPI5 but board SPI5 not enabled"
#  endif
#elif defined(CONFIG_NUCLEO_H753ZI_ST7796_SPI6)
#  define ST7796_SPI_PORTNO 6
#  ifndef CONFIG_STM32H7_SPI6
#    error "ST7796 configured for SPI6 but CONFIG_STM32H7_SPI6 not enabled"
#  endif
#  ifndef CONFIG_NUCLEO_H753ZI_SPI6_ENABLE
#    error "ST7796 configured for SPI6 but board SPI6 not enabled"
#  endif
#else
#  error "No SPI port selected for ST7796"
#endif

/* Verify CONFIG_SPI_CMDDATA is enabled */

#ifndef CONFIG_SPI_CMDDATA
#  error "CONFIG_SPI_CMDDATA must be enabled for ST7796 driver"
#endif

/* Pin configuration from Kconfig with defaults */

#ifndef CONFIG_NUCLEO_H753ZI_ST7796_CS_PIN
#  define CONFIG_NUCLEO_H753ZI_ST7796_CS_PIN "PA4"
#endif

#ifndef CONFIG_NUCLEO_H753ZI_ST7796_DC_PIN
#  define CONFIG_NUCLEO_H753ZI_ST7796_DC_PIN "PA3"
#endif

#ifndef CONFIG_NUCLEO_H753ZI_ST7796_RESET_PIN
#  define CONFIG_NUCLEO_H753ZI_ST7796_RESET_PIN "PA2"
#endif

#ifndef CONFIG_NUCLEO_H753ZI_ST7796_LED_PIN
#  define CONFIG_NUCLEO_H753ZI_ST7796_LED_PIN "PA1"
#endif

#ifndef CONFIG_NUCLEO_H753ZI_ST7796_CS_ACTIVE_LOW
#  define CONFIG_NUCLEO_H753ZI_ST7796_CS_ACTIVE_LOW true
#endif

#ifndef CONFIG_NUCLEO_H753ZI_ST7796_DEVID
#  define CONFIG_NUCLEO_H753ZI_ST7796_DEVID 0
#endif

/* SPI frequency from Kconfig with default */

#ifndef CONFIG_NUCLEO_H753ZI_ST7796_FREQUENCY
#  define CONFIG_NUCLEO_H753ZI_ST7796_FREQUENCY ST7796_SPI_MAXFREQUENCY
#endif

/* Reset timing (from ST7796 datasheet) */

#define ST7796_RESET_DELAY_MS      10
#define ST7796_RESET_HOLD_MS       10
#define ST7796_RESET_RELEASE_MS    120

/* GPIO helper macros */

#define ST7796_GPIO_CONFIG_MASK    0xffff0000
#define ST7796_GPIO_IN_FLOAT       (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz)

/* Display resolution based on orientation from Kconfig */

#if defined(CONFIG_NUCLEO_H753ZI_ST7796_LANDSCAPE) || \
    defined(CONFIG_NUCLEO_H753ZI_ST7796_RLANDSCAPE)
#  define ST7796_XRES              ST7796_YRES_RAW
#  define ST7796_YRES              ST7796_XRES_RAW
#else
#  define ST7796_XRES              ST7796_XRES_RAW
#  define ST7796_YRES              ST7796_YRES_RAW
#endif

/* Base MADCTL value based on orientation and BGR from Kconfig */

#if defined(CONFIG_NUCLEO_H753ZI_ST7796_LANDSCAPE)
#  ifdef CONFIG_NUCLEO_H753ZI_ST7796_BGR
#    define ST7796_MADCTL_BASE     ST7796_MADCTL_LANDSCAPE_BGR
#  else
#    define ST7796_MADCTL_BASE     ST7796_MADCTL_LANDSCAPE
#  endif
#elif defined(CONFIG_NUCLEO_H753ZI_ST7796_RPORTRAIT)
#  ifdef CONFIG_NUCLEO_H753ZI_ST7796_BGR
#    define ST7796_MADCTL_BASE     ST7796_MADCTL_RPORTRAIT_BGR
#  else
#    define ST7796_MADCTL_BASE     ST7796_MADCTL_RPORTRAIT
#  endif
#elif defined(CONFIG_NUCLEO_H753ZI_ST7796_RLANDSCAPE)
#  ifdef CONFIG_NUCLEO_H753ZI_ST7796_BGR
#    define ST7796_MADCTL_BASE     ST7796_MADCTL_RLANDSCAPE_BGR
#  else
#    define ST7796_MADCTL_BASE     ST7796_MADCTL_RLANDSCAPE
#  endif
#else /* Portrait (default) */
#  ifdef CONFIG_NUCLEO_H753ZI_ST7796_BGR
#    define ST7796_MADCTL_BASE     ST7796_MADCTL_PORTRAIT_BGR
#  else
#    define ST7796_MADCTL_BASE     ST7796_MADCTL_PORTRAIT
#  endif
#endif

/* Initial rotation from Kconfig */

#ifdef CONFIG_NUCLEO_H753ZI_ST7796_ROTATION_180
#  define ST7796_INIT_ROTATION     180
#else
#  define ST7796_INIT_ROTATION     0
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_reset_pin;
static uint32_t g_led_pin;
static bool g_st7796_initialized = false;
static FAR struct spi_dev_s *g_spi_dev = NULL;
static FAR struct fb_vtable_s *g_fb_vtable = NULL;

/* Board-specific configuration passed to generic driver */

static struct st7796_config_s g_st7796_config =
{
  .frequency = CONFIG_NUCLEO_H753ZI_ST7796_FREQUENCY,
  .xres      = ST7796_XRES,
  .yres      = ST7796_YRES,
  .bpp       = 16,
  .madctl    =  ST7796_MADCTL_BASE /* 0x28 -> will fix it */,
  .rotation  = ST7796_INIT_ROTATION,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: parse_gpio_pin
 *
 * Description:
 *   Parse GPIO pin string like "PA0" into STM32 GPIO configuration.
 *
 ****************************************************************************/

static uint32_t parse_gpio_pin(FAR const char *pin_str, FAR int *error)
{
  size_t len;
  char port;
  FAR const char *pin_num_str;
  FAR char *endptr;
  long pin_num;
  uint32_t port_base;
  uint32_t gpio_pin;

  *error = 0;

  if (pin_str == NULL)
    {
      *error = -EINVAL;
      return 0;
    }

  while (*pin_str == ' ' || *pin_str == '\t')
    {
      pin_str++;
    }

  len = strlen(pin_str);
  if (len < 3 || len > 4)
    {
      *error = -EINVAL;
      return 0;
    }

  if (pin_str[0] != 'P')
    {
      *error = -EINVAL;
      return 0;
    }

  port = pin_str[1];
  if (port < 'A' || port > 'K')
    {
      *error = -EINVAL;
      return 0;
    }

  pin_num_str = &pin_str[2];
  pin_num = strtol(pin_num_str, &endptr, 10);
  if (*endptr != '\0' || pin_num < 0 || pin_num > 15)
    {
      *error = -EINVAL;
      return 0;
    }

  switch (port)
    {
      case 'A':
       port_base = GPIO_PORTA; break;
      case 'B':
       port_base = GPIO_PORTB; break;
      case 'C':
       port_base = GPIO_PORTC; break;
      case 'D':
       port_base = GPIO_PORTD; break;
      case 'E':
       port_base = GPIO_PORTE; break;
      case 'F':
       port_base = GPIO_PORTF; break;
      case 'G':
       port_base = GPIO_PORTG; break;
      case 'H':
       port_base = GPIO_PORTH; break;
      default:
        *error = -EINVAL;
        return 0;
    }

  switch (pin_num)
    {
      case 0:
        gpio_pin = GPIO_PIN0;  break;
      case 1:
        gpio_pin = GPIO_PIN1;  break;
      case 2:
        gpio_pin = GPIO_PIN2;  break;
      case 3:
        gpio_pin = GPIO_PIN3;  break;
      case 4:
        gpio_pin = GPIO_PIN4;  break;
      case 5:
        gpio_pin = GPIO_PIN5;  break;
      case 6:
        gpio_pin = GPIO_PIN6;  break;
      case 7:
        gpio_pin = GPIO_PIN7;  break;
      case 8:
        gpio_pin = GPIO_PIN8;  break;
      case 9:
        gpio_pin = GPIO_PIN9;  break;
      case 10:
        gpio_pin = GPIO_PIN10; break;
      case 11:
        gpio_pin = GPIO_PIN11; break;
      case 12:
        gpio_pin = GPIO_PIN12; break;
      case 13:
        gpio_pin = GPIO_PIN13; break;
      case 14:
        gpio_pin = GPIO_PIN14; break;
      case 15:
        gpio_pin = GPIO_PIN15; break;
      default:
        *error = -EINVAL;
        return 0;
    }

  return (GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_SPEED_50MHz | GPIO_FLOAT |
          port_base | gpio_pin);
}

/****************************************************************************
 * Name: stm32_st7796_gpio_initialize
 *
 * Description:
 *   Initialize GPIO pins for ST7796 (RESET, LED).
 *
 ****************************************************************************/

static int stm32_st7796_gpio_initialize(void)
{
  int ret;
  int error;

  g_reset_pin = parse_gpio_pin(CONFIG_NUCLEO_H753ZI_ST7796_RESET_PIN,
                               &error);
  if (error != 0)
    {
      syslog(LOG_ERR, "ERROR: Invalid RESET pin '%s': %d\n",
             CONFIG_NUCLEO_H753ZI_ST7796_RESET_PIN, error);
      return error;
    }

  ret = stm32_configgpio(g_reset_pin);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to configure RESET pin: %d\n", ret);
      return ret;
    }

  g_led_pin = parse_gpio_pin(CONFIG_NUCLEO_H753ZI_ST7796_LED_PIN, &error);
  if (error != 0)
    {
      syslog(LOG_ERR, "ERROR: Invalid LED pin '%s': %d\n",
             CONFIG_NUCLEO_H753ZI_ST7796_LED_PIN, error);
      return error;
    }

  ret = stm32_configgpio(g_led_pin);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to configure LED pin: %d\n", ret);
      return ret;
    }

  stm32_gpiowrite(g_reset_pin, true);
  stm32_gpiowrite(g_led_pin, false);  /* Start with backlight OFF */

  return OK;
}

/****************************************************************************
 * Name: stm32_st7796_hardware_reset
 *
 * Description:
 *   Perform hardware reset of ST7796 display.
 *
 ****************************************************************************/

static void stm32_st7796_hardware_reset(void)
{
  stm32_gpiowrite(g_reset_pin, true);
  nxsig_usleep(ST7796_RESET_DELAY_MS * 1000);

  stm32_gpiowrite(g_reset_pin, false);
  nxsig_usleep(ST7796_RESET_HOLD_MS * 1000);

  stm32_gpiowrite(g_reset_pin, true);
  nxsig_usleep(ST7796_RESET_RELEASE_MS * 1000);
}

/****************************************************************************
 * Name: stm32_st7796_spi_initialize
 *
 * Description:
 *   Initialize SPI bus and register CS/DC pins for ST7796.
 *
 ****************************************************************************/

static int stm32_st7796_spi_initialize(void)
{
  int ret;

  stm32_st7796_hardware_reset();

  ret = stm32_spi_register_cs_device(ST7796_SPI_PORTNO,
                                  CONFIG_NUCLEO_H753ZI_ST7796_DEVID,
                                  CONFIG_NUCLEO_H753ZI_ST7796_CS_PIN,
                                  CONFIG_NUCLEO_H753ZI_ST7796_CS_ACTIVE_LOW);

  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register CS: %d\n", ret);
      return ret;
    }

  ret = stm32_spi_register_dc_pin(ST7796_SPI_PORTNO,
                                  CONFIG_NUCLEO_H753ZI_ST7796_DEVID,
                                  CONFIG_NUCLEO_H753ZI_ST7796_DC_PIN);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register DC: %d\n", ret);
      stm32_spi_unregister_cs_device(ST7796_SPI_PORTNO,
                                     CONFIG_NUCLEO_H753ZI_ST7796_DEVID);
      return ret;
    }

  g_spi_dev = stm32_spibus_initialize(ST7796_SPI_PORTNO);
  if (g_spi_dev == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI%d\n",
             ST7796_SPI_PORTNO);
      stm32_spi_unregister_cs_device(ST7796_SPI_PORTNO,
                                     CONFIG_NUCLEO_H753ZI_ST7796_DEVID);
      return -ENODEV;
    }

  g_st7796_initialized = true;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Initialize the framebuffer video hardware. Called by fb_register().
 *
 ****************************************************************************/

int up_fbinitialize(int display)
{
  if (!g_st7796_initialized || g_spi_dev == NULL)
    {
      syslog(LOG_ERR, "ERROR: ST7796 not initialized\n");
      return -ENODEV;
    }

  if (display != 0)
    {
      syslog(LOG_ERR, "ERROR: Invalid display: %d\n", display);
      return -EINVAL;
    }

  g_fb_vtable = st7796_fbinitialize(g_spi_dev, &g_st7796_config);
  if (g_fb_vtable == NULL)
    {
      syslog(LOG_ERR, "ERROR: st7796_fbinitialize() failed\n");
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: up_fbgetvplane
 *
 * Description:
 *   Return a reference to the framebuffer object for the specified plane.
 *
 ****************************************************************************/

FAR struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  if (display != 0 || vplane != 0)
    {
      return NULL;
    }

  return g_fb_vtable;
}

/****************************************************************************
 * Name: up_fbuninitialize
 *
 * Description:
 *   Uninitialize the framebuffer hardware.
 *
 ****************************************************************************/

void up_fbuninitialize(int display)
{
  if (display != 0)
    {
      return;
    }

  stm32_gpiowrite(g_led_pin, false);
  g_fb_vtable = NULL;
}

/****************************************************************************
 * Name: stm32_st7796initialize
 *
 * Description:
 *   Initialize and register the ST7796 LCD driver.
 *   Called from board bringup.
 *
 ****************************************************************************/

int stm32_st7796initialize(int devno)
{
  int ret;

  /* Step 1: Initialize GPIO pins (RESET, LED) */

  ret = stm32_st7796_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: GPIO init failed: %d\n", ret);
      return ret;
    }

  /* Step 2: Initialize SPI bus and register CS/DC */

  ret = stm32_st7796_spi_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: SPI init failed: %d\n", ret);
      return ret;
    }

  /* Step 3: Register framebuffer device
   * This calls up_fbinitialize() which initializes the display
   * and applies initial rotation from config
   */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
      stm32_spi_unregister_cs_device(ST7796_SPI_PORTNO,
                                     CONFIG_NUCLEO_H753ZI_ST7796_DEVID);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_st7796_flush_fb
 *
 * Description:
 *   Flush the entire framebuffer to the display. Call this after
 *   fb_register() to make splashscreen visible on SPI displays.
 *
 ****************************************************************************/

int stm32_st7796_flush_fb(void)
{
  struct fb_area_s area;

  if (g_fb_vtable == NULL || g_fb_vtable->updatearea == NULL)
    {
      syslog(LOG_ERR, "ERROR: Framebuffer not ready for flush\n");
      return -ENODEV;
    }

  area.x = 0;
  area.y = 0;
  area.w = ST7796_XRES;
  area.h = ST7796_YRES;

  return g_fb_vtable->updatearea(g_fb_vtable, &area);
}

/****************************************************************************
 * Name: stm32_st7796_backlight
 *
 * Description:
 *   Control backlight LED.
 *
 ****************************************************************************/

void stm32_st7796_backlight(bool on)
{
  stm32_gpiowrite(g_led_pin, on);
}

/****************************************************************************
 * Name: stm32_st7796_power
 *
 * Description:
 *   Control display power.
 *
 ****************************************************************************/

void stm32_st7796_power(bool on)
{
  if (on)
    {
      stm32_st7796_hardware_reset();
      stm32_gpiowrite(g_led_pin, true);
    }
  else
    {
      stm32_gpiowrite(g_led_pin, false);
    }
}

/****************************************************************************
 * Name: stm32_st7796_reset_display
 *
 * Description:
 *   Public function to reset the display.
 *
 ****************************************************************************/

void stm32_st7796_reset_display(void)
{
  stm32_st7796_hardware_reset();
}

/****************************************************************************
 * Name: stm32_st7796_cleanup
 *
 * Description:
 *   Cleanup ST7796 resources.
 *
 ****************************************************************************/

int stm32_st7796_cleanup(void)
{
  int ret;

  stm32_gpiowrite(g_led_pin, false);

  stm32_configgpio((g_reset_pin & ST7796_GPIO_CONFIG_MASK) |
                   ST7796_GPIO_IN_FLOAT);
  stm32_configgpio((g_led_pin & ST7796_GPIO_CONFIG_MASK) |
                   ST7796_GPIO_IN_FLOAT);

  ret = stm32_spi_unregister_cs_device(ST7796_SPI_PORTNO,
                                       CONFIG_NUCLEO_H753ZI_ST7796_DEVID);
  if (ret < 0)
    {
      syslog(LOG_WARNING, "WARNING: CS unregister failed: %d\n", ret);
    }

  g_st7796_initialized = false;
  g_spi_dev = NULL;
  g_fb_vtable = NULL;

  return ret;
}

#endif /* CONFIG_LCD_ST7796 && CONFIG_NUCLEO_H753ZI_ST7796_ENABLE */

/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/board.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_BOARD_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <sys/boardctl.h>
#include <stdbool.h>

#include "cxd56_clock.h"
#include "cxd56_power.h"
#include "cxd56_flash.h"
#include "cxd56_gauge.h"
#include "cxd56_charger.h"
#include "cxd56_gs2200m.h"
#include "cxd56_i2cdev.h"
#include "cxd56_sdcard.h"
#include "cxd56_wdt.h"
#include "cxd56_gpioif.h"

#include "cxd56_audio.h"
#include "cxd56_altmdm.h"
#include "cxd56_ak09912.h"
#include "cxd56_apds9930.h"
#include "cxd56_apds9960.h"
#include "cxd56_bh1721fvc.h"
#include "cxd56_bh1745nuc.h"
#include "cxd56_bm1383glv.h"
#include "cxd56_bm1422gmv.h"
#include "cxd56_bmi160.h"
#include "cxd56_bmp280.h"
#include "cxd56_kx022.h"
#include "cxd56_lt1pa01.h"
#include "cxd56_rpr0521rs.h"
#include "cxd56_sensors.h"

#include "cxd56_isx012.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking ****************************************************************/

#define BOARD_XTAL_FREQUENCY        (26000000)  /* XTAL oscillator frequency */
#define BOARD_RTCCLK_FREQUENCY      (32768)     /* RTC oscillator frequency */
#define BOARD_INTRCOSC_FREQUENCY    (8192000)   /* Internal RC oscillator frequency */

#ifdef CONFIG_CXD56_80MHz
#  define BOARD_FCLKOUT_FREQUENCY   (80000000)
#else
#  define BOARD_FCLKOUT_FREQUENCY   (97500000)
#endif

#define CXD56_CCLK                  BOARD_FCLKOUT_FREQUENCY

/* USB0 ********************************************************************/

#define BOARD_USB0_CLKSRC           PLL0USB_CLKSEL_XTAL
#define BOARD_USB0_MDIV             0x06167ffa /* Table 149 datsheet, valid for 12Mhz Fclkin */
#define BOARD_USB0_NP_DIV           0x00302062 /* Table 149 datsheet, valid for 12Mhz Fclkin */

/* SPIFI clocking **********************************************************/

/* The SPIFI will receive clocking from a divider per the settings provided
 * in this file.  The NuttX code will configure PLL1 as the input clock
 * for the selected divider
 */

#undef  BOARD_SPIFI_PLL1                        /* No division */
#undef  BOARD_SPIFI_DIVA                        /* Supports division by 1-4 */
#undef  BOARD_SPIFI_DIVB                        /* Supports division by 1-16 */
#undef  BOARD_SPIFI_DIVC                        /* Supports division by 1-16 */
#undef  BOARD_SPIFI_DIVD                        /* Supports division by 1-16 */
#undef  BOARD_SPIFI_DIVE                        /* Supports division by 1-256 */

#if BOARD_FCLKOUT_FREQUENCY < 20000000
#  define BOARD_SPIFI_PLL1          1           /* Use PLL1 directly */
#else
#  define BOARD_SPIFI_DIVB          1           /* Use IDIVB */
#endif

/* We need to configure the divider so that its output is as close to the
 * desired SCLK value.  The peak data transfer rate will be about half of
 * this frequency in bytes per second.
 */

#if BOARD_FCLKOUT_FREQUENCY < 20000000
#  define BOARD_SPIFI_FREQUENCY     BOARD_FCLKOUT_FREQUENCY  /* 72Mhz? */
#else
#  define BOARD_SPIFI_DIVIDER       (14)        /* 204MHz / 14 = 14.57MHz */
#  define BOARD_SPIFI_FREQUENCY     (102000000) /* 204MHz / 14 = 14.57MHz */
#endif

/* UART clocking ***********************************************************/

/* Configure all UARTs to use the XTAL input frequency */

#define BOARD_UART0_BASEFREQ        BOARD_XTAL_FREQUENCY
#define BOARD_UART1_BASEFREQ        48750000
#define BOARD_UART2_BASEFREQ        BOARD_XTAL_FREQUENCY

/* LED definitions *********************************************************/

#define GPIO_LED1           (PIN_I2S1_BCK)
#define GPIO_LED2           (PIN_I2S1_LRCK)
#define GPIO_LED3           (PIN_I2S1_DATA_IN)
#define GPIO_LED4           (PIN_I2S1_DATA_OUT)

#define BOARD_LED1          (0)
#define BOARD_LED2          (1)
#define BOARD_LED3          (2)
#define BOARD_LED4          (3)
#define BOARD_NLEDS         (4)

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT      (1 << BOARD_LED1)
#define BOARD_LED2_BIT      (1 << BOARD_LED2)
#define BOARD_LED3_BIT      (1 << BOARD_LED3)
#define BOARD_LED4_BIT      (1 << BOARD_LED4)

/* LED pattern for use with board_autoled_on() and board_autoled_off() */

#define LED_STARTED             (BOARD_LED1_BIT)
#define LED_HEAPALLOCATE        (BOARD_LED2_BIT)
#define LED_IRQSENABLED         (BOARD_LED1_BIT | BOARD_LED2_BIT)
#define LED_STACKCREATED        (BOARD_LED3_BIT)
#define LED_INIRQ               (BOARD_LED1_BIT | BOARD_LED3_BIT)
#define LED_SIGNAL              (BOARD_LED2_BIT | BOARD_LED3_BIT)
#define LED_ASSERTION           (BOARD_LED1_BIT | BOARD_LED2_BIT | BOARD_LED3_BIT)
#define LED_PANIC               (BOARD_LED4_BIT)

/* Buttons definitions *****************************************************/

#define BOARD_NUM_BUTTONS   (2)

/* Power Control definitions ***********************************************/

/*     Switch    Device
 *     --------- -------------------------------
 *     LSW2      AcaPulco Audio Digital VDD
 *     LSW3      SPI-Flash & TCXO
 *     LSW4      GNSS LNA
 *     GPO0      AcaPulco Audio Analog VDD
 *     GPO1      Sensor 1.8V
 *     GPO2      Sensor 3.3V
 *     GPO3      Bluetooth/Bluetooth Low Energy
 *     GPO4      Image Sensor 1.2V
 *     GPO5      Image Sensor 3.3V
 *     GPO6      eMMC 3.3V/1.8V
 *     GPO7      Image Sensor 1.8V
 *
 */

#define PMIC_NONE           (0)
#define PMIC_TYPE_LSW       (1u << 8)
#define PMIC_TYPE_GPO       (1u << 9)
#define PMIC_TYPE_DDCLDO    (1u << 10)
#define PMIC_GET_TYPE(v)    ((v) & 0xff00)
#define PMIC_GET_CH(v)      ((v) & 0x00ff)
#define PMIC_LSW(n)         (PMIC_TYPE_LSW | (1u << (n)))
#define PMIC_GPO(n)         (PMIC_TYPE_GPO | (1u << (n)))
#define PMIC_DDCLDO(n)      (PMIC_TYPE_DDCLDO | (1u << (n)))

enum board_power_device
{
  /* DDC/LDO */

  POWER_DDC_IO          = PMIC_DDCLDO(0),
  POWER_LDO_EMMC        = PMIC_DDCLDO(1),
  POWER_DDC_ANA         = PMIC_DDCLDO(2),
  POWER_LDO_ANA         = PMIC_DDCLDO(3),
  POWER_DDC_CORE        = PMIC_DDCLDO(4),
  POWER_LDO_PERI        = PMIC_DDCLDO(5),

  /* Load Switch */

  POWER_AUDIO_DVDD      = PMIC_LSW(2),
  POWER_FLASH           = PMIC_LSW(3),
  POWER_TCXO            = PMIC_LSW(4),
  POWER_LNA             = PMIC_LSW(4),

  /* GPO */

  POWER_AUDIO_AVDD      = PMIC_GPO(1),
  POWER_AUDIO_MUTE      = PMIC_GPO(6),
  POWER_SENSOR_18V      = PMIC_GPO(1),
  POWER_SENSOR_33V      = PMIC_GPO(2),
  POWER_BMI160          = POWER_SENSOR_18V,
  POWER_SENSOR          = POWER_SENSOR_18V | POWER_SENSOR_33V,
  POWER_BTBLE           = PMIC_GPO(3),
  POWER_EINK            = PMIC_NONE,
  POWER_EMMC            = PMIC_GPO(6),
  POWER_LFOUR           = PMIC_NONE,
  POWER_LTE             = PMIC_NONE,
  POWER_IMAGE_SENSOR    = PMIC_GPO(4) | PMIC_GPO(5) | PMIC_GPO(7),
};

/* CXD5247 audio control definitions ***************************************/

#define CXD5247_XRST  PIN_SPI3_CS2_X
#define CXD5247_AVDD  (0x01)
#define CXD5247_DVDD  (0x02)

/* LCD Display clocking ****************************************************/

#define ILI9340_SPI_MAXFREQUENCY    40000000

/* Display device pin definitions ******************************************/

#if defined(CONFIG_LCD_ON_MAIN_BOARD) /* Display connected to main board. */

#define DISPLAY_RST     PIN_I2S0_BCK
#define DISPLAY_DC      PIN_I2S0_LRCK

#define DISPLAY_SPI     5

#else /* Display is connected through extension board. */

#define DISPLAY_RST     PIN_SPI2_MISO
#define DISPLAY_DC      PIN_PWM2

#define DISPLAY_SPI     4

#endif

/* Sensor device bus definitions *******************************************/

#define SENSOR_I2C      0
#define SENSOR_SPI      3

/* Imager device pin definitions *******************************************/

#define IMAGER_RST      PIN_SDIO_DIR1_3
#define IMAGER_SLEEP    PIN_SDIO_DIR0

#define IMAGER_I2C      2

/* Set signal id for notify USB device connection status and
 * supply current value.
 * signal returns "usbdev_notify_s" struct pointer in sival_ptr.
 *
 * Arg: Value of sinal number
 */

#define BOARDIOC_USBDEV_SETNOTIFYSIG      (BOARDIOC_USER+0x0001)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_boardinitialize
 *
 * Description:
 *
 ****************************************************************************/

void cxd56_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_BOARD_H */

/****************************************************************************
 * configs/spresense/include/board.h
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

#ifndef _CONFIGS_SPRESENSE_INCLUDE_BOARD_H
#define _CONFIGS_SPRESENSE_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <sys/boardctl.h>
#include <stdbool.h>

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

#define BOARD_LED1          (0)
#define BOARD_LED2          (1)
#define BOARD_NLEDS         (2)

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT      (1 << BOARD_LED1)
#define BOARD_LED2_BIT      (1 << BOARD_LED2)

/* LED pattern for use with board_autoled_on() and board_autoled_off()
 *           ON            OFF
 *       LED1   LED2   LED1   LED2
 * PTN0: OFF    OFF     -      -
 * PTN1: ON     OFF     -      -
 * PTN2: -      ON      -      OFF
 *
 */

#define LED_AUTOLED_PTN0    (0)
#define LED_AUTOLED_PTN1    (1)
#define LED_AUTOLED_PTN2    (2)

#define LED_STARTED         (LED_AUTOLED_PTN0)
#define LED_HEAPALLOCATE    (LED_AUTOLED_PTN1)
#define LED_IRQSENABLED     (LED_AUTOLED_PTN1)
#define LED_STACKCREATED    (LED_AUTOLED_PTN1)
#define LED_INIRQ           (LED_AUTOLED_PTN2)
#define LED_SIGNAL          (LED_AUTOLED_PTN2)
#define LED_ASSERTION       (LED_AUTOLED_PTN2)
#define LED_PANIC           (LED_AUTOLED_PTN2)

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
  POWER_TCXO            = PMIC_LSW(3),
  POWER_LNA             = PMIC_LSW(4),

  /* GPO */

  POWER_AUDIO_AVDD      = PMIC_GPO(0),
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
#endif  /* _CONFIGS_SPRESENSE_INCLUDE_BOARD_H */

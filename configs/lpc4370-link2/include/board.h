#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_GPIO_IRQ)
#  include <nuttx/irq.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking ****************************************************************/
/* NOTE:  The following definitions require lpc43_cgu.h.  It is not included
 * here because the including C file may not have that file in its include
 * path.
 *
 * The Xplorer board has four crystals on board:
 *
 *     Y1 - RTC 32.768 MHz oscillator input,
 *     Y2 - 24.576 MHz input to the UDA 1380 audio codec,
 *     Y3 - 12.000 MHz LPC43xx crystal oscillator input
 *     Y4 - 50 MHz input for Ethernet
 */

#define BOARD_XTAL_FREQUENCY        (12000000)  /* XTAL oscillator frequency (Y3) */
#define BOARD_RTCCLK_FREQUENCY      (32768)     /* RTC oscillator frequency (Y1) */
#define BOARD_INTRCOSC_FREQUENCY    (4000000)   /* Internal RC oscillator frequency */

/* Integer and direct modes are supported:
 *
 * In integer mode (Fclkout < 156000000):
 *    Fclkin  = BOARD_XTAL_FREQUENCY
 *    Fclkout = Msel * FClkin / Nsel
 *    Fcco    = 2 * Psel * Fclkout
 * In direct mode (Fclkout > 156000000):
 *    Fclkin  = BOARD_XTAL_FREQUENCY
 *    Fclkout = Msel * FClkin / Nsel
 *    Fcco    = Fclkout
 */

#ifdef CONFIG_LPC43_72MHz

/* NOTE:  At 72MHz, the calibrated value of CONFIG_BOARD_LOOPSPERMSEC was
 * determined to be:
 *
 *    CONFIG_BOARD_LOOPSPERMSEC=7191
 *
 * executing from SRAM.
 */

/* Final clocking (Integer mode with no ramp-up)
 *
 *    Fclkout = 6 * 12MHz / 1  = 72MHz
 *    Fcco    = 2 * 2 * 72MHz = 216MHz
 */

#  define BOARD_PLL_MSEL            (6)         /* Msel = 6 */
#  define BOARD_PLL_NSEL            (1)         /* Nsel = 1 */
#  define BOARD_PLL_PSEL            (2)         /* Psel = 2 */

#  define BOARD_FCLKOUT_FREQUENCY   (72000000)  /* 6 * 12,000,000 / 1 */
#  define BOARD_FCCO_FREQUENCY      (244000000) /* 2 * 2 * Fclkout */

#else

/* NOTE:  At 72MHz, the calibrated value of CONFIG_BOARD_LOOPSPERMSEC was
 * determined to be:
 *
 *    CONFIG_BOARD_LOOPSPERMSEC=18535
 *
 * executing from SRAM.
 */

/* Intermediate ramp-up clocking (Integer mode). If BOARD_PLL_RAMP_MSEL
 * is not defined, there will be no ramp-up.
 *
 *    Fclkout = 9 * 12MHz / 1  = 108MHz
 *    Fcco    = 2 * 1 * 108MHz = 216MHz
 */

#  define BOARD_PLL_RAMP_MSEL       (9)         /* Msel = 9 */
#  define BOARD_PLL_RAMP_NSEL       (1)         /* Nsel = 1 */
#  define BOARD_PLL_RAMP_PSEL       (1)         /* Psel = 1 */

#  define BOARD_RAMP_FCLKOUT        (108000000) /* 9 * 12,000,000 / 1 */
#  define BOARD_RAMP_FCCO           (216000000) /* 2 * 1 * Fclkout */

/* Final clocking (Direct mode).
 *
 *    Fclkout = 17 * 12MHz / 1 = 204MHz
 *    Fcco    = Fclockout      = 204MHz
 */

#  define BOARD_PLL_MSEL            (17)        /* Msel = 17 */
#  define BOARD_PLL_NSEL            (1)         /* Nsel = 1 */

#  define BOARD_FCLKOUT_FREQUENCY   (204000000) /* 17 * 12,000,000 / 1 */
#  define BOARD_FCCO_FREQUENCY      (204000000) /* Fclockout */

#endif



#define LPC43_CCLK                  BOARD_FCLKOUT_FREQUENCY

#if defined(CONFIG_LPC43_BUS) || defined(CONFIG_LPC43_MCPWM) || defined(CONFIG_LPC43_I2C0) || defined(CONFIG_LPC43_I2S0) || defined(CONFIG_LPC43_I2S1)  || defined(CONFIG_LPC43_CAN1)
#define BOARD_ABP1_CLKSRC			BASE_APB1_CLKSEL_XTAL
#define BOARD_ABP1_FREQUENCY		BOARD_XTAL_FREQUENCY
#endif


#if defined(CONFIG_LPC43_BUS) || defined(CONFIG_LPC43_I2C1) || defined(CONFIG_LPC43_DAC) || defined(CONFIG_LPC43_ADC0) || defined(CONFIG_LPC43_ADC1)  || defined(CONFIG_LPC43_CAN0)
#define BOARD_ABP3_CLKSRC			BASE_APB3_CLKSEL_XTAL
#define BOARD_ABP3_FREQUENCY		BOARD_XTAL_FREQUENCY
#endif



#define BOARD_IDIVA_DIVIDER			(2)
#define BOARD_IDIVA_CLKSRC          IDIVA_CLKSEL_PLL1
#define BOARD_IDIVA_FREQUENCY		(BOARD_FCLKOUT_FREQUENCY/BOARD_IDIVA_DIVIDER)


/* USB0 ********************************************************************/
/* Settings needed in lpc43_cpu.c */

#define BOARD_USB0_CLKSRC			PLL0USB_CLKSEL_XTAL
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
/* Configure all U[S]ARTs to use the XTAL input frequency */

#define BOARD_USART0_CLKSRC         BASE_USART0_CLKSEL_XTAL
#define BOARD_USART0_BASEFREQ       BOARD_XTAL_FREQUENCY

#define BOARD_UART1_CLKSRC          BASE_UART1_CLKSEL_XTAL
#define BOARD_UART1_BASEFREQ        BOARD_XTAL_FREQUENCY

#define BOARD_USART2_CLKSRC         BASE_USART2_CLKSEL_XTAL
#define BOARD_USART2_BASEFREQ       BOARD_XTAL_FREQUENCY

#define BOARD_USART3_CLKSRC         BASE_USART3_CLKSEL_XTAL
#define BOARD_USART3_BASEFREQ       BOARD_XTAL_FREQUENCY


/* SSP clocking ***********************************************************
 *
 * BOARD_SSPX_BASEFREQ may be further divided by 2-254 to get the SSP clock.  If we
 * want a usable range of 400KHz to 25MHz for the SSP, then:
 *
 * 1. SSPCLK must be greater than (2*25MHz) = 50MHz, and
 * 2. SSPCLK must be less than (254*400Khz) = 101.6MHz.
 *
 */


#define BOARD_SSP0_CLKSRC         BASE_SSP0_CLKSEL_IDIVA
#define BOARD_SSP0_BASEFREQ       BOARD_IDIVA_FREQUENCY

#define BOARD_SSP1_CLKSRC         BASE_SSP1_CLKSEL_IDIVA
#define BOARD_SSP1_BASEFREQ       BOARD_IDIVA_FREQUENCY



/* LED definitions *********************************************************/

/* 
 *  LED1   K2  GPIO0[8]
 *
 * LED index values for use with lpc43_setled()
 */

#define BOARD_LED        0
#define BOARD_NLEDS      1

/* LED bits for use with lpc43_setleds() */

#define BOARD_LED_BIT    (1 << BOARD_LED)

/* If CONFIG_ARCH_LEDS is defined, the LEDs will be controlled as follows
 * for NuttX debug functionality (where NC means "No Change"). If
 * CONFIG_ARCH_LEDS is not defined, then the LEDs are completely under
 * control of the application.  The following interfaces are then available
 * for application control of the LEDs:
 *
 *  void lpc43_ledinit(void);
 *  void lpc43_setled(int led, bool ledon);
 *  void lpc43_setleds(uint8_t ledset);
 */
                                      /* LED      */
#define LED_STARTED                0  /* OFF      */
#define LED_HEAPALLOCATE           0  /* OFF      */
#define LED_IRQSENABLED            0  /* OFF      */
#define LED_STACKCREATED           1  /* ON       */
#define LED_INIRQ                  2  /* NC       */
#define LED_SIGNAL                 2  /* NC       */
#define LED_ASSERTION              2  /* NC       */
#define LED_PANIC                  3  /* Flashing */

/* UART Pins ****************************************************************/
/* 
 * The following definitions must be provided so that the LPC43 serial
 * driver can set up the U[S]ART for the serial console properly (see the
 * file arch/arc/src/lpc43xx/lpc43*_pinconf.h for more info).
 */

#define PINCONF_U0_TXD  PINCONF_U0_TXD_3
#define PINCONF_U0_RXD  PINCONF_U0_RXD_3
#define PINCONF_U0_DIR  PINCONF_U0_DIR_3

#define PINCONF_U1_TXD  PINCONF_U1_TXD_1
#define PINCONF_U1_RXD  PINCONF_U1_RXD_1

#define PINCONF_U2_TXD  PINCONF_U2_TXD_2
#define PINCONF_U2_RXD  PINCONF_U2_RXD_2
#define PINCONF_U2_DIR  PINCONF_U2_DIR_2

#define PINCONF_U3_TXD  PINCONF_U3_TXD_2
#define PINCONF_U3_RXD  PINCONF_U3_RXD_2
#define PINCONF_U3_DIR  PINCONF_U3_DIR_2

//I2C1 pins, not really accessible on the board
#define PINCONF_I2C1_SCL PINCONF_I2C1_SCL_1
#define PINCONF_I2C1_SDA PINCONF_I2C1_SDA_1

//SSP1 pins
#define PINCONF_SSP1_MISO PINCONF_SSP1_MISO_3
#define PINCONF_SSP1_MOSI PINCONF_SSP1_MOSI_3
#define PINCONF_SSP1_SCK  PINCONF_SSP1_SCK_1
#define PINCONF_SSP1_SSEL PINCONF_SSP1_SSEL_1


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
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_boardinitialize
 *
 * Description:
 *   All LPC43xx architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ****************************************************************************/

EXTERN void lpc43_boardinitialize(void);

/****************************************************************************
 * Name:  lpc43_ledinit, lpc43_setled, and lpc43_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.  If CONFIG_ARCH_LEDS is not defined, then the following interfaces
 *   are available to control the LEDs from user applications.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
EXTERN void lpc43_ledinit(void);
EXTERN void lpc43_setled(int led, bool ledon);
EXTERN void lpc43_setleds(uint8_t ledset);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_BOARD_BOARD_H */

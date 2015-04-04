/************************************************************************************
 * configs/samv71-xult/src/samv71-xult.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 ************************************************************************************/

#ifndef __CONFIGS_SAMV71_XULT_SRC_SAMV71_XULT_H
#define __CONFIGS_SAMV71_XULT_SRC_SAMV71_XULT_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#define HAVE_HSMCI       1
#define HAVE_AUTOMOUNTER 1
#define HAVE_USB         1
#define HAVE_USBDEV      1
#define HAVE_USBMONITOR  1
#define HAVE_NETWORK     1
#define HAVE_MACADDR     1
#define HAVE_MTDCONFIG   1
#define HAVE_WM8904      1
#define HAVE_AUDIO_NULL  1

/* HSMCI */
/* Can't support MMC/SD if the card interface is not enabled */

#if !defined(CONFIG_SAMV7_HSMCI0)
#  undef HAVE_HSMCI
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(HAVE_HSMCI) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No MMC/SD support
#  undef HAVE_HSMCI
#endif

/* We need PIO interrupts on GPIOD to support card detect interrupts */

#if defined(HAVE_HSMCI) && !defined(CONFIG_SAMV7_GPIOD_IRQ)
#  warning PIOD interrupts not enabled.  No MMC/SD support.
#  undef HAVE_HSMCI
#endif

/* MMC/SD minor numbers */

#ifndef CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

#ifndef CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDSLOTNO 0
#endif

#if CONFIG_NSH_MMCSDMINOR != 0
#  error SAMV71 has only one MMC/SD slot (CONFIG_NSH_MMCSDMINOR)
#  undef CONFIG_NSH_MMCSDSLOTNO
#  define CONFIG_NSH_MMCSDSLOTNO 0
#endif

#define HSMCI0_SLOTNO CONFIG_NSH_MMCSDSLOTNO
#define HSMCI0_MINOR  CONFIG_NSH_MMCSDMINOR

/* Automounter.  Currently only works with HSMCI. */

#if !defined(CONFIG_FS_AUTOMOUNTER) || !defined(HAVE_HSMCI)
#  undef HAVE_AUTOMOUNTER
#  undef CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT
#endif

#ifndef CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT
#  undef HAVE_AUTOMOUNTER
#endif

#ifdef HAVE_AUTOMOUNTER
#  ifdef CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT
  /* HSMCI0 Automounter defaults */

#    ifndef CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT_FSTYPE
#      define CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT_FSTYPE "vfat"
#    endif

#    ifndef CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT_BLKDEV
#      define CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT_BLKDEV "/dev/mmcds0"
#    endif

#    ifndef CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT_MOUNTPOINT
#      define CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT_MOUNTPOINT "/mnt/sdcard0"
#    endif

#    ifndef CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT_DDELAY
#      define CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT_DDELAY 1000
#    endif

#    ifndef CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT_UDELAY
#      define CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT_UDELAY 2000
#    endif
#  endif /* CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT */
#endif /* HAVE_AUTOMOUNTER */

/* USB Device */
/* CONFIG_SAMV7_UDP and CONFIG_USBDEV must be defined, or there is no USB
 * device.
 */

#if !defined(CONFIG_SAMV7_UDP) || !defined(CONFIG_USBDEV)
#  undef HAVE_USB
#  undef HAVE_USBDEV
#endif

/* Check if we should enable the USB monitor before starting NSH */

#ifndef HAVE_USBDEV
#  undef CONFIG_USBDEV_TRACE
#endif

#if !defined(CONFIG_SYSTEM_USBMONITOR) || !defined(CONFIG_USBDEV_TRACE)
#  undef HAVE_USBMONITOR
#endif

/* Networking and AT24-based MTD config */

#if !defined(CONFIG_NET) || !defined(CONFIG_SAMV7_EMAC)
#  undef HAVE_NETWORK
#  undef HAVE_MACADDR
#endif

#if !defined(CONFIG_SAMV7_TWIHS0) || !defined(CONFIG_MTD_AT24XX)
#  undef HAVE_MACADDR
#  undef HAVE_MTDCONFIG
#endif

#if defined(CONFIG_NSH_NOMAC) || !defined(CONFIG_AT24XX_EXTENDED)
#  undef HAVE_MACADDR
#endif

#if !defined(CONFIG_MTD_CONFIG)
#  undef HAVE_MTDCONFIG
#endif

/* Audio */
/* PCM/WM8904 driver */

#ifndef CONFIG_AUDIO_WM8904
#  undef HAVE_WM8904
#endif

#ifdef HAVE_WM8904
#  ifndef CONFIG_SAMV7_TWIHS0
#    warning CONFIG_SAMV7_TWIHS0 is required for audio support
#    undef HAVE_WM8904
#  endif

#  ifndef CONFIG_SAMV7_SSC0
#    warning CONFIG_SAMV7_SSC0 is required for audio support
#    undef HAVE_WM8904
#  endif

#  if !defined(CONFIG_SAMV7_GPIOD_IRQ)
#    warning CONFIG_SAMV7_GPIOD_IRQ is required for audio support
#    undef HAVE_WM8904
#  endif

#  ifndef CONFIG_AUDIO_FORMAT_PCM
#    warning CONFIG_AUDIO_FORMAT_PCM is required for audio support
#    undef HAVE_WM8904
#  endif

#  ifndef CONFIG_SAMV71XULT_WM8904_I2CFREQUENCY
#    warning Defaulting to maximum WM8904 I2C frequency
#    define CONFIG_SAMV71XULT_WM8904_I2CFREQUENCY 400000
#  endif

#  if CONFIG_SAMV71XULT_WM8904_I2CFREQUENCY > 400000
#    warning WM8904 I2C frequency cannot exceed 400KHz
#    undef CONFIG_SAMV71XULT_WM8904_I2CFREQUENCY
#    define CONFIG_SAMV71XULT_WM8904_I2CFREQUENCY 400000
#  endif
#endif

/* PCM/null driver */

#ifndef CONFIG_AUDIO_NULL
#  undef HAVE_AUDIO_NULL
#endif

#ifndef HAVE_WM8904
#  undef HAVE_AUDIO_NULL
#endif

#ifdef HAVE_AUDIO_NULL
#  ifndef CONFIG_AUDIO_FORMAT_PCM
#    warning CONFIG_AUDIO_FORMAT_PCM is required for audio support
#    undef HAVE_AUDIO_NULL
#  endif
#endif

/* SAMV71-XULT GPIO Pin Definitions *************************************************/

/* Ethernet MAC.
 *
 * KSZ8061RNBVA Connections
 * ------------------------
 *
 *   ------ --------- --------- --------------------------
 *   SAMV71 SAMV71    Ethernet  Shared functionality
 *   Pin    Function  Function
 *   ------ --------- --------- --------------------------
 *   PD00   GTXCK     REF_CLK   Shield
 *   PD01   GTXEN     TXEN
 *   PD02   GTX0      TXD0
 *   PD03   GTX1      TXD1
 *   PD04   GRXDV     CRS_DV    Trace
 *   PD05   GRX0      RXD0      Trace
 *   PD06   GRX1      RXD1      Trace
 *   PD07   GRXER     RXER      Trace
 *   PD08   GMDC      MDC       Trace
 *   PD09   GMDIO     MDIO
 *   PA19   GPIO      INTERRUPT EXT1, Shield
 *   PA29   GPIO      SIGDET
 *   PC10   GPIO      RESET
 *   ------ --------- --------- --------------------------
 */

#define GPIO_EMAC0_INT    (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                           GPIO_INT_FALLING | GPIO_PORT_PIOA | GPIO_PIN19)
#define GPIO_EMAC0_SIGDET (GPIO_INPUT | GPIO_CFG_DEFAULT | \
                           GPIO_PORT_PIOA | GPIO_PIN29)
#define GPIO_EMAC0_RESET  (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_SET | \
                           GPIO_PORT_PIOC | GPIO_PIN10)

#define IRQ_EMAC0_INT     SAM_IRQ_PA19

/* LEDs
 *
 * There are two yellow LED available on the SAM V71 Xplained Ultra board that
 * can be turned on and off.  The LEDs can be activated by driving the
 * connected I/O line to GND.
 *
 *   ------ ----------- ---------------------
 *   SAMV71 Function    Shared functionality
 *   PIO
 *   ------ ----------- ---------------------
 *   PA23   Yellow LED0 EDBG GPIO
 *   PC09   Yellow LED1 LCD, and Shield
 *   ------ ----------- ---------------------
 */

#define GPIO_LED0     (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                       GPIO_PORT_PIOA | GPIO_PIN23)
#define GPIO_LED1     (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                       GPIO_PORT_PIOC | GPIO_PIN9)

/* Buttons
 *
 * SAM V71 Xplained Ultra contains three mechanical buttons. One button is the
 * RESET button connected to the SAM V71 reset line and the others are generic
 * user configurable buttons. When a button is pressed it will drive the I/O
 * line to GND.
 *
 *   ------ ----------- ---------------------
 *   SAMV71 Function    Shared functionality
 *   PIO
 *   ------ ----------- ---------------------
 *   RESET  RESET       Trace, Shield, and EDBG
 *   PA09   SW0         EDBG GPIO and Camera
 *   PB12   SW1         EDBG SWD and Chip Erase
 *   ------ ----------- ---------------------
 *
 * NOTES:
 *
 *   - There are no pull-up resistors connected to the generic user buttons so
 *     it is necessary to enable the internal pull-up in the SAM V71 to use the
 *     button.
 *   - PB12 is set up as a system flash ERASE pin when the firmware boots. To
 *     use the SW1, PB12 has to be configured as a normal regular I/O pin in
 *     the MATRIX module. For more information see the SAM V71 datasheet.
 */

#define GPIO_SW0      (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                       GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN9)
#define GPIO_SW1      (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                       GPIO_INT_BOTHEDGES | GPIO_PORT_PIOB | GPIO_PIN12)

#define IRQ_SW0       SAM_IRQ_PA9
#define IRQ_SW1       SAM_IRQ_PB12

/* HSMCI SD Card Detect
 *
 * The SAM V71 Xplained Ultra has one standard SD card connector which is
 * connected to the High Speed Multimedia Card Interface (HSMCI) of the SAM
 * V71. SD card connector:
 *
 *   ------ ----------------- ---------------------
 *   SAMV71 SAMV71            Shared functionality
 *   Pin    Function
 *   ------ ----------------- ---------------------
 *   PA30   MCDA0 (DAT0)
 *   PA31   MCDA1 (DAT1)
 *   PA26   MCDA2 (DAT2)
 *   PA27   MCDA3 (DAT3)      Camera
 *   PA25   MCCK (CLK)        Shield
 *   PA28   MCCDA (CMD)
 *   PD18   Card Detect (C/D) Shield
 *   ------ ----------------- ---------------------
 */

#define GPIO_MCI0_CD (GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
                      GPIO_INT_BOTHEDGES | GPIO_PORT_PIOD | GPIO_PIN18)
#define IRQ_MCI0_CD   SAM_IRQ_PD18

/* USB Host
 *
 * The SAM V71 Xplained Ultra has a Micro-USB connector for use with the SAM V71
 * USB module labeled as TARGET USB on the kit. In USB host mode VBUS voltage is
 * provided by the kit and has to be enabled by setting the "VBUS Host Enable"
 * pin (PC16) low.
 */

#define GPIO_VBUSON (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                     GPIO_PORT_PIOC | GPIO_PIN16)

/* WM8904 Audio Codec ***************************************************************/
/* SAMV71 Interface        WM8904 Interface
 * ---- ------------ ------- ----------------------------------
 * PIO  Usage        Pin     Function
 * ---- ------------ ------- ----------------------------------
 * PA3  TWD0         SDA     I2C control interface, data line
 * PA4  TWCK0        SCLK    I2C control interface, clock line
 * PA10 RD           ADCDAT  Digital audio output (microphone)
 * PB18 PCK2         MCLK    Master clock
 * PB0  TF           LRCLK   Left/right data alignment clock
 * PB1  TK           BCLK    Bit clock, for synchronization
 * PD11 GPIO         IRQ     Audio interrupt
 * PD24 RF           LRCLK   Left/right data alignment clock
 * PD26 TD           DACDAT  Digital audio input (headphone)
 * ---- ------------ ------- ----------------------------------
 */

/* Audio Interrupt. All interrupts are default, active high level.  Pull down
 * internally in the WM8904.  So we want no pull-up/downs and we want to
 * interrupt on the high level.
 */

#define GPIO_INT_WM8904    (PIO_INPUT | PIO_CFG_DEFAULT | PIO_CFG_DEGLITCH | \
                            PIO_INT_HIGHLEVEL | PIO_PORT_PIOD | PIO_PIN11)
#define IRQ_INT_WM8904     SAM_IRQ_PD11

/* The MW8904 communicates on TWI0, I2C address 0x1a for control operations */

#define WM8904_TWI_BUS     0
#define WM8904_I2C_ADDRESS 0x1a

/* The MW8904 transfers data on SSC0 */

#define WM8904_SSC_BUS     0

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_sdram_config
 *
 * Description:
 *   Configures the on-board SDRAM.  SAMV71 Xplained Ultra features one external
 *   IS42S16100E-7BLI, 512Kx16x2, 10ns, SDRAM. SDRAM0 is connected to chip select
 *   NCS1.
 *
 *  Input Parameters:
 *     None
 *
 *  Assumptions:
 *    The DDR memory regions is configured as strongly ordered memory.  When we
 *    complete initialization of SDRAM and it is ready for use, we will make DRAM
 *    into normal memory.
 *
 ************************************************************************************/

#ifdef CONFIG_SAMV7_SDRAMC
void sam_sdram_config(void);
#else
#  define sam_sdram_config(t)
#endif

/************************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Bring up board features
 *
 ************************************************************************************/

#if defined(CONFIG_NSH_ARCHINIT) || defined(CONFIG_BOARD_INITIALIZE)
int sam_bringup(void);
#endif

/************************************************************************************
 * Name: sam_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SAM4E-EK board.
 *
 ************************************************************************************/

void sam_spiinitialize(void);

/************************************************************************************
 * Name: sam_hsmci_initialize
 *
 * Description:
 *   Initialize HSMCI support
 *
 ************************************************************************************/

#ifdef HAVE_HSMCI
int sam_hsmci_initialize(int slot, int minor);
#else
# define sam_hsmci_initialize(s,m) (-ENOSYS)
#endif

/************************************************************************************
 * Name:  sam_usbinitialize
 *
 * Description:
 *   Called from stm32_boardinitialize very early in initialization to setup USB-
 *   related GPIO pins for the SAMV71-XULT board.
 *
 ************************************************************************************/

#ifdef HAVE_USB
void sam_usbinitialize(void);
#endif

/************************************************************************************
 * Name: sam_netinitialize
 *
 * Description:
 *   Configure board resources to support networking.
 *
 ************************************************************************************/

#ifdef HAVE_NETWORK
void sam_netinitialize(void);
#endif

/************************************************************************************
 * Name: sam_emac0_setmac
 *
 * Description:
 *   Read the Ethernet MAC address from the AT24 FLASH and configure the Ethernet
 *   driver with that address.
 *
 ************************************************************************************/

#ifdef HAVE_MACADDR
int sam_emac0_setmac(void);
#endif

/************************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ************************************************************************************/

#ifdef HAVE_HSMCI
bool sam_cardinserted(int slotno);
#else
#  define sam_cardinserted(slotno) (false)
#endif

/************************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if the card in the MMCSD slot is write protected
 *
 ************************************************************************************/

#ifdef HAVE_HSMCI
bool sam_writeprotected(int slotno);
#endif

/************************************************************************************
 * Name:  sam_automount_initialize
 *
 * Description:
 *   Configure auto-mounters for each enable and so configured HSMCI
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ************************************************************************************/

#ifdef HAVE_AUTOMOUNTER
void sam_automount_initialize(void);
#endif

/************************************************************************************
 * Name:  sam_automount_event
 *
 * Description:
 *   The HSMCI card detection logic has detected an insertion or removal event.  It
 *   has already scheduled the MMC/SD block driver operations.  Now we need to
 *   schedule the auto-mount event which will occur with a substantial delay to make
 *   sure that everything has settle down.
 *
 * Input Parameters:
 *   slotno - Identifies the HSMCI0 slot: HSMCI0 or HSMCI1_SLOTNO.  There is a
 *      terminology problem here:  Each HSMCI supports two slots, slot A and slot B.
 *      Only slot A is used.  So this is not a really a slot, but an HSCMI peripheral
 *      number.
 *   inserted - True if the card is inserted in the slot.  False otherwise.
 *
 *  Returned Value:
 *    None
 *
 *  Assumptions:
 *    Interrupts are disabled.
 *
 ************************************************************************************/

#ifdef HAVE_AUTOMOUNTER
void sam_automount_event(int slotno, bool inserted);
#endif

/************************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if the card in the MMCSD slot is write protected
 *
 ************************************************************************************/

#ifdef HAVE_HSMCI
bool sam_writeprotected(int slotno);
#else
#  define sam_writeprotected(slotno) (false)
#endif

/************************************************************************************
 * Name: sam_at24config
 *
 * Description:
 *   Create an AT24xx-based MTD configuration device for storage device configuration
 *   information.
 *
 ************************************************************************************/

#ifdef HAVE_MTDCONFIG
int sam_at24config(void);
#endif

/****************************************************************************
 * Name: sam_wm8904_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the WM8904 device.  This function will register the driver
 *   as /dev/wm8904[x] where x is determined by the minor device number.
 *
 * Input Parameters:
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef HAVE_WM8904
int sam_wm8904_initialize(int minor);
#endif /* HAVE_WM8904 */

/****************************************************************************
 * Name: sam_audio_null_initialize
 *
 * Description:
 *   Set up to use the NULL audio device for PCM unit-level testing.
 *
 * Input Parameters:
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef HAVE_AUDIO_NULL
int sam_audio_null_initialize(int minor);
#endif /* HAVE_AUDIO_NULL */

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SAMV71_XULT_SRC_SAMV71_XULT_H */

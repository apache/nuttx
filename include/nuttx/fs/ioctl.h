/****************************************************************************
 * include/nuttx/fs/ioctl.h
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

#ifndef __INCLUDE_NUTTX_FS_IOCTL_H
#define __INCLUDE_NUTTX_FS_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* General ioctl definitions ************************************************/

/* Each NuttX ioctl commands are uint16_t's consisting of an 8-bit type
 * identifier and an 8-bit command number.  All command type identifiers are
 * defined below:
 */

#define _TIOCBASE       (0x0100) /* Terminal I/O ioctl commands */
#define _WDIOCBASE      (0x0200) /* Watchdog driver ioctl commands */
#define _FIOCBASE       (0x0300) /* File system ioctl commands */
#define _DIOCBASE       (0x0400) /* Character driver ioctl commands */
#define _BIOCBASE       (0x0500) /* Block driver ioctl commands */
#define _MTDIOCBASE     (0x0600) /* MTD ioctl commands */
#define _SIOCBASE       (0x0700) /* Socket ioctl commands */
#define _ARPIOCBASE     (0x0800) /* ARP ioctl commands */
#define _TSIOCBASE      (0x0900) /* Touchscreen ioctl commands */
#define _SNIOCBASE      (0x0a00) /* Sensor ioctl commands */
#define _ANIOCBASE      (0x0b00) /* Analog (DAC/ADC) ioctl commands */
#define _PWMIOCBASE     (0x0c00) /* PWM ioctl commands */
#define _CAIOCBASE      (0x0d00) /* CDC/ACM ioctl commands */
#define _BATIOCBASE     (0x0e00) /* Battery driver ioctl commands */
#define _QEIOCBASE      (0x0f00) /* Quadrature encoder ioctl commands */
#define _AUDIOIOCBASE   (0x1000) /* Audio ioctl commands */
#define _LCDIOCBASE     (0x1100) /* LCD character driver ioctl commands */
#define _SLCDIOCBASE    (0x1200) /* Segment LCD ioctl commands */
                                 /* 0x1300: Not used */
#define _WLCIOCBASE     (0x1400) /* Wireless modules ioctl character driver commands */
#define _CFGDIOCBASE    (0x1500) /* Config Data device (app config) ioctl commands */
#define _TCIOCBASE      (0x1600) /* Timer ioctl commands */
#define _JOYBASE        (0x1700) /* Joystick ioctl commands */
#define _PIPEBASE       (0x1800) /* FIFO/pipe ioctl commands */
#define _RTCBASE        (0x1900) /* RTC ioctl commands */
#define _RELAYBASE      (0x1a00) /* Relay devices ioctl commands */
#define _CANBASE        (0x1b00) /* CAN ioctl commands */
#define _BTNBASE        (0x1c00) /* Button ioctl commands */
#define _ULEDBASE       (0x1d00) /* User LED ioctl commands */
#define _ZCBASE         (0x1e00) /* Zero Cross ioctl commands */
#define _LOOPBASE       (0x1f00) /* Loop device commands */
#define _MODEMBASE      (0x2000) /* Modem ioctl commands */
#define _I2CBASE        (0x2100) /* I2C driver commands */
#define _SPIBASE        (0x2200) /* SPI driver commands */
#define _GPIOBASE       (0x2300) /* GPIO driver commands */
#define _CLIOCBASE      (0x2400) /* Contactless modules ioctl commands */
#define _USBCBASE       (0x2500) /* USB-C controller ioctl commands */
#define _MAC802154BASE  (0x2600) /* 802.15.4 MAC ioctl commands */
#define _PWRBASE        (0x2700) /* Power-related ioctl commands */
#define _FBIOCBASE      (0x2800) /* Frame buffer character driver ioctl commands */
#define _NXTERMBASE     (0x2900) /* NxTerm character driver ioctl commands */
#define _RFIOCBASE      (0x2a00) /* RF devices ioctl commands */
#define _RPTUNBASE      (0x2b00) /* Remote processor tunnel ioctl commands */
#define _NOTECTLBASE    (0x2c00) /* Note filter control ioctl commands*/
#define _NOTERAMBASE    (0x2d00) /* Noteram device ioctl commands*/
#define _RCIOCBASE      (0x2e00) /* Remote Control device ioctl commands */
#define _HIMEMBASE      (0x2f00) /* Himem device ioctl commands*/
#define _EFUSEBASE      (0x3000) /* Efuse device ioctl commands*/
#define _MTRIOBASE      (0x3100) /* Motor device ioctl commands*/
#define _WLIOCBASE      (0x8b00) /* Wireless modules ioctl network commands */

/* boardctl() commands share the same number space */

#define _BOARDBASE      (0xff00) /* boardctl commands */

/* Macros used to manage ioctl commands.  IOCTL commands are divided into
 * groups of 256 commands for major, broad functional areas.  That makes
 * them a limited resource.
 */

#define _IOC_MASK       (0x00ff)
#define _IOC_TYPE(cmd)  ((cmd) & ~_IOC_MASK)
#define _IOC_NR(cmd)    ((cmd) & _IOC_MASK)

#define _IOC(type,nr)   ((type)|(nr))

/* Terminal I/O ioctl commands **********************************************/

#define _TIOCVALID(c)   (_IOC_TYPE(c)==_TIOCBASE)
#define _TIOC(nr)       _IOC(_TIOCBASE,nr)

/* Terminal I/O IOCTL definitions are retained in tioctl.h */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/serial/tioctl.h>

/* Watchdog driver ioctl commands *******************************************/

#define _WDIOCVALID(c)  (_IOC_TYPE(c)==_WDIOCBASE)
#define _WDIOC(nr)      _IOC(_WDIOCBASE,nr)

/* NuttX file system ioctl definitions **************************************/

#define _FIOCVALID(c)   (_IOC_TYPE(c)==_FIOCBASE)
#define _FIOC(nr)       _IOC(_FIOCBASE,nr)

#define FIOC_MMAP       _FIOC(0x0001)     /* IN:  Location to return address (void **)
                                           * OUT: If media is directly accessible,
                                           *      return (void*) base address
                                           *      of file
                                           */
#define FIOC_REFORMAT   _FIOC(0x0002)     /* IN:  None
                                           * OUT: None
                                           */
#define FIOC_OPTIMIZE   _FIOC(0x0003)     /* IN:  The number of bytes to recover
                                           *      (ignored on most file systems)
                                           * OUT: None
                                           */
#define FIOC_FILENAME   _FIOC(0x0004)     /* IN:  FAR const char ** pointer
                                           * OUT: Pointer to a persistent file name
                                           *      (Guaranteed to persist while the
                                           *      file is open).
                                           */
#define FIOC_INTEGRITY  _FIOC(0x0005)     /* Run a consistency check on the
                                           *      file system media.
                                           * IN:  None
                                           * OUT: None */
#define FIOC_DUMP       _FIOC(0x0006)     /* Dump logical content of media.
                                           * IN:  None
                                           * OUT: None */

#define FIONREAD        _FIOC(0x0007)     /* IN:  Location to return value (int *)
                                           * OUT: Bytes readable from this fd
                                           */
#define FIONWRITE       _FIOC(0x0008)     /* IN:  Location to return value (int *)
                                           * OUT: Number bytes in send queue
                                           */
#define FIONSPACE       _FIOC(0x0009)     /* IN:  Location to return value (int *)
                                           * OUT: Free space in send queue.
                                           */
#define FIONUSERFS      _FIOC(0x000a)     /* IN:  Pointer to struct usefs_config_s
                                           *      holding userfs configuration.
                                           * OUT: Instance number is returned on
                                           *      success.
                                           */
#define FIONBIO         _FIOC(0x000b)     /* IN:  Boolean option takes an
                                           *      int value.
                                           * OUT: Origin option.
                                           */
#define FIOC_MINOR      _FIOC(0x000c)     /* IN:  None
                                           * OUT: Integer that contains device
                                           *      minor number
                                           */
#define FIOCLEX         _FIOC(0x000d)     /* IN:  None
                                           * OUT: None
                                           */
#define FIONCLEX        _FIOC(0x000e)     /* IN:  None
                                           * OUT: None
                                           */
#define FIOC_FILEPATH   _FIOC(0x000f)     /* IN:  FAR char *(length >= PATH_MAX)
                                           * OUT: The full file path
                                           */

/* NuttX file system ioctl definitions **************************************/

#define _DIOCVALID(c)   (_IOC_TYPE(c)==_DIOCBASE)
#define _DIOC(nr)       _IOC(_DIOCBASE,nr)

#define DIOC_GETPRIV    _DIOC(0x0001)     /* IN:  Location to return handle (void **)
                                           * OUT: Reference to internal data
                                           *      structure.  May have a reference
                                           *      incremented.
                                           */
#define DIOC_RELPRIV    _DIOC(0x0003)     /* IN:  None
                                           * OUT: None, reference obtained by
                                           *      FIOC_GETPRIV released.
                                           */

#define DIOC_SETKEY     _DIOC(0X0004)     /* IN:  Encryption key
                                           * OUT: None
                                           */

/* NuttX block driver ioctl definitions *************************************/

#define _BIOCVALID(c)   (_IOC_TYPE(c)==_BIOCBASE)
#define _BIOC(nr)       _IOC(_BIOCBASE,nr)

#define BIOC_XIPBASE    _BIOC(0x0001)     /* Perform mapping to random access memory.
                                           * IN:  Pointer to pointer to void in
                                           *      which to received the XIP base.
                                           * OUT: If media is directly accessible,
                                           *      return (void *) base address
                                           *      of device memory */
#define BIOC_PROBE      _BIOC(0x0002)     /* Re-probe and interface; check for media
                                           * in the slot
                                           * IN:  None
                                           * OUT: None (ioctl return value provides
                                           *      success/failure indication). */
#define BIOC_EJECT      _BIOC(0x0003)     /* Eject/disable media in the slot
                                           * IN:  None
                                           * OUT: None (ioctl return value provides
                                           *      success/failure indication). */
#define BIOC_LLFORMAT   _BIOC(0x0004)     /* Low-Level Format on SMART flash devices
                                           * IN:  None
                                           * OUT: None (ioctl return value provides
                                           *      success/failure indication). */
#define BIOC_GETFORMAT  _BIOC(0x0005)     /* Returns SMART flash format information
                                           * such as format status, logical sector
                                           * size, total sectors, free sectors, etc.
                                           * IN:  None
                                           * OUT: Pointer to the format information. */
#define BIOC_ALLOCSECT  _BIOC(0x0006)     /* Allocate a logical sector from the block
                                           * device.
                                           * IN:  None
                                           * OUT: Logical sector number allocated. */
#define BIOC_FREESECT   _BIOC(0x0007)     /* Allocate a logical sector from the block
                                           * device.
                                           * IN:  None
                                           * OUT: Logical sector number allocated. */
#define BIOC_READSECT   _BIOC(0x0008)     /* Read a logical sector from the block
                                           * device.
                                           * IN:  Pointer to sector read data (the
                                           *      logical sector number, count and
                                           *      read buffer address
                                           * OUT: Number of bytes read or error */
#define BIOC_WRITESECT  _BIOC(0x0009)     /* Write to data to a logical sector
                                           * IN:  Pointer to sector write data (the
                                           *      logical sector number and write
                                           *      buffer address
                                           * OUT: None (ioctl return value provides
                                           *      success/failure indication). */
#define BIOC_GETPROCFSD _BIOC(0x000a)     /* Get ProcFS data specific to the
                                           * block device.
                                           * IN:  Pointer to a struct defined for
                                           *      the block to load with it's
                                           *      ProcFS data.
                                           * OUT: None (ioctl return value provides
                                           *      success/failure indication). */
#define BIOC_DEBUGCMD   _BIOC(0x000b)     /* Send driver specific debug command /
                                           * data to the block device.
                                           * IN:  Pointer to a struct defined for
                                           *      the block with specific debug
                                           *      command and data.
                                           * OUT: None.  */
#define BIOC_GEOMETRY   _BIOC(0x000c)     /* Used only by BCH to return the
                                           * geometry of the contained block
                                           * driver.
                                           * IN:  Pointer to writable instance
                                           *      of struct geometry in which
                                           *      to return geometry.
                                           * OUT: Data return in user-provided
                                           *      buffer. */
#define BIOC_FLUSH      _BIOC(0x000d)     /* Flush the block device write buffer
                                           * IN:  None
                                           * OUT: None (ioctl return value provides
                                           *      success/failure indication). */

/* NuttX MTD driver ioctl definitions ***************************************/

#define _MTDIOCVALID(c)   (_IOC_TYPE(c)==_MTDIOCBASE)
#define _MTDIOC(nr)       _IOC(_MTDIOCBASE,nr)

/* Socket IOCTLs ************************************************************/

/* See include/nuttx/net/ioctl.h */

#define _SIOCVALID(c)    (_IOC_TYPE(c)==_SIOCBASE)
#define _SIOC(nr)        _IOC(_SIOCBASE,nr)

/* NuttX ARP driver ioctl definitions (see netinet/arp.h) *******************/

#define _ARPIOCVALID(c)   (_IOC_TYPE(c)==_ARPIOCBASE)
#define _ARPIOC(nr)       _IOC(_ARPIOCBASE,nr)

/* NuttX touchscreen ioctl definitions (see nuttx/input/touchscreen.h) ******/

#define _TSIOCVALID(c)    (_IOC_TYPE(c)==_TSIOCBASE)
#define _TSIOC(nr)        _IOC(_TSIOCBASE,nr)

/* NuttX sensor ioctl definitions (see nuttx/sensor/ioctl.h) ****************/

#define _SNIOCVALID(c)    (_IOC_TYPE(c)==_SNIOCBASE)
#define _SNIOC(nr)        _IOC(_SNIOCBASE,nr)

/* NuttX Analog (DAC/ADC) ioctl commands (see nuttx/analog/ioctl.h **********/

#define _ANIOCVALID(c)    (_IOC_TYPE(c)==_ANIOCBASE)
#define _ANIOC(nr)        _IOC(_ANIOCBASE,nr)

/* NuttX PWM ioctl definitions (see nuttx/timers/pwm.h) *********************/

#define _PWMIOCVALID(c)   (_IOC_TYPE(c)==_PWMIOCBASE)
#define _PWMIOC(nr)       _IOC(_PWMIOCBASE,nr)

/* NuttX USB CDC/ACM serial driver ioctl definitions ************************/

/* (see nuttx/usb/cdcacm.h) */

#define _CAIOCVALID(c)    (_IOC_TYPE(c)==_CAIOCBASE)
#define _CAIOC(nr)        _IOC(_CAIOCBASE,nr)

/* NuttX battery driver ioctl definitions ***********************************/

/* (see nuttx/power/battery.h) */

#define _BATIOCVALID(c)   (_IOC_TYPE(c)==_BATIOCBASE)
#define _BATIOC(nr)       _IOC(_BATIOCBASE,nr)

/* NuttX Quadrature Encoder driver ioctl definitions ************************/

/* (see nuttx/power/battery.h) */

#define _QEIOCVALID(c)    (_IOC_TYPE(c)==_QEIOCBASE)
#define _QEIOC(nr)        _IOC(_QEIOCBASE,nr)

/* NuttX Audio driver ioctl definitions *************************************/

/* (see nuttx/audio/audio.h) */

#define _AUDIOIOCVALID(c) (_IOC_TYPE(c)==_AUDIOIOCBASE)
#define _AUDIOIOC(nr)     _IOC(_AUDIOIOCBASE,nr)

/* LCD character driver ioctl definitions ***********************************/

/* (see nuttx/include/lcd/slcd_codec.h */

#define _LCDIOCVALID(c)   (_IOC_TYPE(c)==_LCDIOCBASE)
#define _LCDIOC(nr)       _IOC(_LCDIOCBASE,nr)

/* Segment LCD driver ioctl definitions *************************************/

/* (see nuttx/include/lcd/slcd_codec.h */

#define _SLCDIOCVALID(c)  (_IOC_TYPE(c)==_SLCDIOCBASE)
#define _SLCDIOC(nr)      _IOC(_SLCDIOCBASE,nr)

/* Wireless driver character driver ioctl definitions ***********************/

/* (see nuttx/include/wireless/ioctl.h */

#define _WLCIOCVALID(c)   (_IOC_TYPE(c)==_WLCIOCBASE)
#define _WLCIOC(nr)       _IOC(_WLCIOCBASE,nr)

/* Application Config Data driver ioctl definitions *************************/

/* (see nuttx/include/configdata.h */

#define _CFGDIOCVALID(c)  (_IOC_TYPE(c)==_CFGDIOCBASE)
#define _CFGDIOC(nr)      _IOC(_CFGDIOCBASE,nr)

/* Timer driver ioctl commands **********************************************/

/* (see nuttx/include/timer.h */

#define _TCIOCVALID(c)    (_IOC_TYPE(c)==_TCIOCBASE)
#define _TCIOC(nr)        _IOC(_TCIOCBASE,nr)

/* Joystick driver ioctl definitions ****************************************/

/* Discrete Joystick (see nuttx/include/input/djoystick.h */

#define _JOYIOCVALID(c)   (_IOC_TYPE(c)==_JOYBASE)
#define _JOYIOC(nr)       _IOC(_JOYBASE,nr)

/* FIFOs and pipe driver ioctl definitions **********************************/

#define _PIPEIOCVALID(c)  (_IOC_TYPE(c)==_PIPEBASE)
#define _PIPEIOC(nr)      _IOC(_PIPEBASE,nr)

#define PIPEIOC_POLICY    _PIPEIOC(0x0001)  /* Set buffer policy
                                             * IN: unsigned long integer
                                             *     0=free on last close
                                             *       (default)
                                             *     1=fre when empty
                                             * OUT: None */

/* RTC driver ioctl definitions *********************************************/

/* (see nuttx/include/rtc.h */

#define _RTCIOCVALID(c)   (_IOC_TYPE(c)==_RTCBASE)
#define _RTCIOC(nr)       _IOC(_RTCBASE,nr)

/* Relay driver ioctl definitions *******************************************/

/* (see nuttx/power/relay.h */

#define _RELAYIOCVALID(c) (_IOC_TYPE(c)==_RELAYBASE)
#define _RELAYIOC(nr)     _IOC(_RELAYBASE,nr)

/* CAN driver ioctl definitions *********************************************/

/* (see nuttx/can/can.h */

#define _CANIOCVALID(c)   (_IOC_TYPE(c)==_CANBASE)
#define _CANIOC(nr)       _IOC(_CANBASE,nr)

/* Button driver ioctl definitions ******************************************/

/* (see nuttx/input/buttons.h */

#define _BTNIOCVALID(c)   (_IOC_TYPE(c)==_BTNBASE)
#define _BTNIOC(nr)       _IOC(_BTNBASE,nr)

/* User LED driver ioctl definitions ****************************************/

/* (see nuttx/leds/usersled.h */

#define _ULEDIOCVALID(c)  (_IOC_TYPE(c)==_ULEDBASE)
#define _ULEDIOC(nr)      _IOC(_ULEDBASE,nr)

/* Zero Cross driver ioctl definitions **************************************/

/* (see nuttx/include/sensor/zerocross.h */

#define _ZCIOCVALID(c)    (_IOC_TYPE(c)==_ZCBASE)
#define _ZCIOC(nr)        _IOC(_ZCBASE,nr)

/* Loop driver ioctl definitions ********************************************/

/* (see nuttx/include/fs/loop.h */

#define _LOOPIOCVALID(c)  (_IOC_TYPE(c)==_LOOPBASE)
#define _LOOPIOC(nr)      _IOC(_LOOPBASE,nr)

/* Modem driver ioctl definitions *******************************************/

/* see nuttx/include/modem/ioctl.h */

#define _MODEMIOCVALID(c) (_IOC_TYPE(c)==_MODEMBASE)
#define _MODEMIOC(nr)     _IOC(_MODEMBASE,nr)

/* I2C driver ioctl definitions *********************************************/

/* see nuttx/include/i2c/i2c_master.h */

#define _I2CIOCVALID(c)   (_IOC_TYPE(c)==_I2CBASE)
#define _I2CIOC(nr)       _IOC(_I2CBASE,nr)

/* SPI driver ioctl definitions *********************************************/

/* see nuttx/include/spi/spi_transfer.h */

#define _SPIIOCVALID(c)   (_IOC_TYPE(c)==_SPIBASE)
#define _SPIIOC(nr)       _IOC(_SPIBASE,nr)

/* GPIO driver command definitions ******************************************/

/* see nuttx/include/ioexpander/gpio.h */

#define _GPIOCVALID(c)    (_IOC_TYPE(c)==_GPIOBASE)
#define _GPIOC(nr)        _IOC(_GPIOBASE,nr)

/* Contactless driver ioctl definitions *************************************/

/* (see nuttx/include/contactless/ioctl.h */

#define _CLIOCVALID(c)    (_IOC_TYPE(c)==_CLIOCBASE)
#define _CLIOC(nr)        _IOC(_CLIOCBASE,nr)

/* USB-C controller driver ioctl definitions ********************************/

/* (see nuttx/include/usb/xxx.h */

#define _USBCIOCVALID(c)  (_IOC_TYPE(c)==_USBCBASE)
#define _USBCIOC(nr)      _IOC(_USBCBASE,nr)

/* 802.15.4 MAC driver ioctl definitions ************************************/

/* (see nuttx/include/wireless/ieee802154/ieee802154_mac.h */

#define _MAC802154IOCVALID(c)  (_IOC_TYPE(c)==_MAC802154BASE)
#define _MAC802154IOC(nr)      _IOC(_MAC802154BASE,nr)

/* Power-Related IOCTLs *****************************************************/

#define _PWRIOCVALID(c)   (_IOC_TYPE(c)==_PWRBASE)
#define _PWRIOC(nr)       _IOC(_PWRBASE,nr)

/* Frame buffer character drivers *******************************************/

#define _FBIOCVALID(c)    (_IOC_TYPE(c)==_FBIOCBASE)
#define _FBIOC(nr)        _IOC(_FBIOCBASE,nr)

/* NxTerm character drivers *************************************************/

#define _NXTERMVALID(c)   (_IOC_TYPE(c)==_NXTERMBASE)
#define _NXTERMIOC(nr)    _IOC(_NXTERMBASE,nr)

/* NuttX RF ioctl definitions (see nuttx/rf/ioctl.h) ************************/

#define _RFIOCVALID(c)    (_IOC_TYPE(c)==_RFIOCBASE)
#define _RFIOC(nr)        _IOC(_RFIOCBASE,nr)

/* Rptun drivers ************************************************************/

#define _RPTUNIOCVALID(c)   (_IOC_TYPE(c)==_RPTUNBASE)
#define _RPTUNIOC(nr)       _IOC(_RPTUNBASE,nr)

/* Notectl drivers **********************************************************/

#define _NOTECTLIOCVALID(c) (_IOC_TYPE(c) == _NOTECTLBASE)
#define _NOTECTLIOC(nr)     _IOC(_NOTECTLBASE, nr)

/* Noteram drivers **********************************************************/

#define _NOTERAMIOCVALID(c) (_IOC_TYPE(c) == _NOTERAMBASE)
#define _NOTERAMIOC(nr)     _IOC(_NOTERAMBASE, nr)

/* Remote Control drivers ***************************************************/

#define _RCIOCVALID(c)    (_IOC_TYPE(c)==_RCIOCBASE)
#define _RCIOC(nr)        _IOC(_RCIOCBASE,nr)

/* Hime drivers *************************************************************/

#define _HIMEMIOCVALID(c)   (_IOC_TYPE(c) == _HIMEMBASE)
#define _HIMEMIOC(nr)       _IOC(_HIMEMBASE, nr)

/* Efuse drivers ************************************************************/

#define _EFUSEIOCVALID(c)   (_IOC_TYPE(c) == _EFUSEBASE)
#define _EFUSEIOC(nr)       _IOC(_EFUSEBASE, nr)

/* Motor drivers ************************************************************/

#define _MTRIOCVALID(c)     (_IOC_TYPE(c) == _MTRIOBASE)
#define _MTRIOC(nr)         _IOC(_MTRIOBASE, nr)

/* Wireless driver network ioctl definitions ********************************/

/* (see nuttx/include/wireless/wireless.h */

#define _WLIOCVALID(c)    (_IOC_TYPE(c)==_WLIOCBASE)
#define _WLIOC(nr)        _IOC(_WLIOCBASE,nr)

/* boardctl() command definitions *******************************************/

#define _BOARDIOCVALID(c) (_IOC_TYPE(c)==_BOARDBASE)
#define _BOARDIOC(nr)     _IOC(_BOARDBASE,nr)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_FS_IOCTL_H */

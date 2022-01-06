/****************************************************************************
 * boards/arm/sama5/jti-toucan2/src/jti-toucan.h
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_SAMA5_JTI_TOUCAN2_SRC_JTI_TOUCAN2_H
#define __BOARDS_ARM_SAMA5_JTI_TOUCAN2_SRC_JTI_TOUCAN2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/irq.h>
#include <nuttx/irq.h>
#include "sam_spi.h"
#include "sam_flexcom_spi.h"
#include <sam_classd.h>
#include "hardware/sam_pinmap.h"
//#include "sam_pio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

//#define HAVE_USBHOST  1
#define HAVE_USBDEV     1
#define HAVE_USBMONITOR 1
#define HAVE_M25P       1
#define HAVE_AT25       1
//#define HAVE_CAN		1
//#define HAVE_PWM		1
#define HAVE_EGT 1
#define HAVE_BLUETOOTH 1
#define HAVE_AUDIO_CLASSD 1
#define HAVE_AUDIO_NULL 1
//#define HAVE_FUSB302

#define BOARD_CRYSTAL_FREQUENCY_12MHZ  0
#define BOARD_CRYSTAL_FREQUENCY_16MHZ  1
#define BOARD_CRYSTAL_FREQUENCY_24MHZ  2
#define BOARD_CRYSTAL_FREQUENCY_48MHZ  2

#define BOARD_CRYSTAL_FREQUENCY BOARD_CRYSTAL_FREQUENCY_24MHZ

/* M25P Serial FLASH */

/* Can't support the M25P device if SPI0 or M25P support are not enabled */

#if !defined(CONFIG_SAMA5_SPI0) || !defined(CONFIG_MTD_M25P)
#  undef HAVE_M25P
#  warning HAVEM25P has been undefined
#endif

/* Can't support M25P features if mountpoints are disabled or if we were not
 * asked to mount the M25P part
 */
#if 0 /* while I work out what file system!) */
#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_SAMA5D27_M25P_AUTOMOUNT)
#  undef HAVE_M25P
#  warning HAVEM25P has been undefined
#endif

/* If we are going to mount the M25P, then they user must also have told
 * us what to do with it by setting one of these.
 */

#ifndef CONFIG_FS_NXFFS
#  undef CONFIG_SAMA5D27_M25P_NXFFS
#  warning HAVEM25P has been undefined
#endif

#if !defined(CONFIG_SAMA5D27_M25P_FTL) && !defined(CONFIG_SAMA5D27_M25P_NXFFS)
#  undef HAVE_M25P
#  warning HAVEM25P has been undefined
#endif

#if defined(CONFIG_SAMA5D27_M25P_FTL) && defined(CONFIG_SAMA5D27_M25P_NXFFS)
#  warning Both CONFIG_SAMA5D2XULT_M25P_FTL and CONFIG_SAMA5D2XULT_M25P_NXFFS are set
#  warning Ignoring CONFIG_SAMA5D2XULT_M25P_NXFFS
#  undef CONFIG_SAMA5D2XULT_M25P_NXFFS
#endif
#endif

/* Assign minor device numbers.  For example, if we also use MINOR number 0
 * for the M25P, it should appear as /dev/mtdblock0
 */

#ifdef HAVE_M25P
#  define M25P_MINOR  0
//int sam_at25_automount(int minor);
#endif


/* USB Host / USB Device */

/* Either CONFIG_SAMA5_UHPHS or CONFIG_SAMA5_UDPHS must be defined,
 * or there is no USB of any kind.
 */

#if !defined(CONFIG_SAMA5_UHPHS)
#  undef CONFIG_SAMA5_OHCI
#  undef CONFIG_SAMA5_EHCI
#endif

#if !defined(CONFIG_SAMA5_UDPHS)
#  undef HAVE_USBDEV
#endif

/* CONFIG_USBDEV and CONFIG_USBHOST must also be defined */

#if !defined(CONFIG_USBDEV)
#  undef HAVE_USBDEV
#endif

#if defined(CONFIG_USBHOST)
#  if !defined(CONFIG_SAMA5_EHCI) && !defined(CONFIG_SAMA5_OHCI)
#    warning CONFIG_USBHOST is defined, but neither CONFIG_SAMA5_OHCI nor CONFIG_SAMA5_EHCI are defined
#  endif
#else
#  undef CONFIG_SAMA5_OHCI
#  undef CONFIG_SAMA5_EHCI
#endif

#if !defined(CONFIG_SAMA5_OHCI) || !defined(CONFIG_SAMA5_EHCI)
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#if !defined(CONFIG_USBMONITOR)
#  undef HAVE_USBMONITOR
#endif

#ifndef HAVE_USBDEV
#  undef CONFIG_USBDEV_TRACE
#endif

#ifndef HAVE_USBHOST
#  undef CONFIG_USBHOST_TRACE
#endif

#if !defined(CONFIG_USBDEV_TRACE) && !defined(CONFIG_USBHOST_TRACE)
#else
#  undef HAVE_USBMONITOR
#endif


/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define SAMA5_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define SAMA5_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* Board-specific PIO */



/* JTi Toucan2 pinout
 *
 * There is one main connector, J2.
 *
 *
 * Pin 1	Ground
 * Pin 2	CAN2 HI
 * Pin 3	CAN2 LO
 * Pin 4	RS232 TX 
 * Pin 5	0-12V switch input1, active high 
 * Pin 6	0-12V switch input4, active high 
 * Pin 7	0-12V switch input3, active high 
 * Pin 8	0-12V switch input5, active high 
 * Pin 9 	0-5V Analogue Input 2 (ADC 5)
 * Pin 10 	0-5V Analogue Input 3 (ADC 6)
 * Pin 11	K-thermocouple input+ (onboard MAX31855K)
 * Pin 12	Lowside driver output (onboard 1A lowside driver driven by PWM)
 * Pin 13	Kline/Lin (onboard TJA1021)
 * Pin 14	Power in (8-40V DC)
 * Pin 15 	CAN1 HI
 * Pin 16	CAN1 LO
 * Pin 17	RS232 RX
 * Pin 18	0-5V output1, controlled from a filtered/buffered processor PWM output
 * Pin 19	0-5V output2, controlled from a filtered/buffered processor PWM output
 * Pin 20	0-12V switch input2, active high 
 * Pin 21	0-12V switch input6, active high 
 * Pin 22 	0-5V Analogue Input 1 (ADC 4)
 * Pin 23 	0-5V Analogue Input 4 (ADC 7)
 * Pin 24	K-thermocouple input - (onboard MAX31855K)
 * Pin 25	5V reference output from onboard linear regulator
 * Pin 26	Ground (typically used to power off-board sensors along with +5V from pin 25)

*/ 




/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int sam_bringup(void);

/****************************************************************************
 * Name: sam_usbinitialize
 *
 * Description:
 *   Called from sam_usbinitialize very early in initialization to setup
 *   USB-related PIO pins for the board.
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_UDPHS) || defined(CONFIG_SAMA5_UHPHS)
void weak_function sam_usbinitialize(void);
#endif

/****************************************************************************
 * Name: sam_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ****************************************************************************/

#ifdef HAVE_USBHOST
int sam_usbhost_initialize(void);
#endif

#ifdef HAVE_USBDEV

#endif

#if defined(CONFIG_PWM)
int sam_pwm_setup(int pwm_channel);
#endif

#if defined(CONFIG_SAMA5_MCAN0) || defined(CONFIG_SAMA5_MCAN1)
void board_can_pio_control_initialize(void);
#define PIO_MCAN0_SILENT_MODE (PIO_OUTPUT |  PIO_CFG_DEFAULT | \
        PIO_OUTPUT_CLEAR | PIO_PORT_PIOC | PIO_PIN22)

#define PIO_MCAN1_SILENT_MODE (PIO_OUTPUT |  PIO_CFG_DEFAULT | \
        PIO_OUTPUT_CLEAR | PIO_PORT_PIOD | PIO_PIN13)

#define PIO_MCAN0_TERMINATION_ENABLE (PIO_OUTPUT | \
       PIO_CFG_DEFAULT | PIO_OUTPUT_SET | PIO_PORT_PIOA | PIO_PIN12)

#define PIO_MCAN1_TERMINATION_ENABLE (PIO_OUTPUT | \
       PIO_CFG_DEFAULT | PIO_OUTPUT_SET | PIO_PORT_PIOA | PIO_PIN13)
int sam_can_setup(void);
#endif

#if defined(HAVE_AT25) 
#define PIO_AT25_NPCS0 (PIO_OUTPUT | PIO_CFG_PULLUP | PIO_OUTPUT_SET | \
                        PIO_PORT_PIOA | PIO_PIN17)
#define AT25_PORT       SPI0_CS0
#endif

#if defined(HAVE_M25P)
#define PIO_M25P_NPCS1 (PIO_OUTPUT | PIO_CFG_PULLUP | PIO_OUTPUT_SET | \
                        PIO_PORT_PIOA | PIO_PIN18)
#define M25P_PORT       SPI0_CS1
int board_usbmsc_initialize(int port);
#endif

#if defined (HAVE_EGT)
# if !defined (CONFIG_SAMA5_FLEXCOM4_SPI)
#   warning CONFIG_SAMA5_FLEXCOM4_SPI was not defined so HAVE_EGT has been undefined
#   undef HAVE_EGT
# else
#   define PIO_FLEXCOM4_SPI_MISO PIO_FLEXCOM4_IO1_1 
#   define PIO_FLEXCOM4_SPI_MOSI PIO_FLEXCOM4_IO0_1 
#   define PIO_FLEXCOM4_SPI_SPCK PIO_FLEXCOM4_IO2_1 
#   define PIO_EGT_NPCS1 PIO_FLEXCOM4_IO4_1 

#   define EGT_PORT FLEXCOM4_SPI_CS1
# endif
#endif

#if defined (HAVE_BLUETOOTH)
# if !defined (CONFIG_SAMA5_FLEXCOM1_SPI)
#   warning CONFIG_SAMA5_FLEXCOM1_SPI was not defined so HAVE_BLUETOOTH has been undefined
#   undef HAVE_BLUETOOTH
# else
#   define PIO_FLEXCOM1_SPI_MISO PIO_FLEXCOM1_IO1 
#   define PIO_FLEXCOM1_SPI_MOSI PIO_FLEXCOM1_IO0 
#   define PIO_FLEXCOM1_SPI_SPCK PIO_FLEXCOM1_IO2 
#   define PIO_BLUETOOTH_NPCS0 PIO_FLEXCOM1_IO3 

#   define BLUETOOTH_PORT FLEXCOM1_SPI_CS0
# endif
#endif

/* FUSB302 */
#if defined (HAVE_FUSB302)
#define PIO_FUSB302_INT \
           (PIO_INPUT | PIO_CFG_DEGLITCH | \
            PIO_INT_BOTHEDGES | PIO_PORT_PIOA | PIO_PIN29 )
          
#define IRQ_FUSB302_INT SAM_IRQ_PA29
#else /*unused pin */
#define PIO_FUSB302_INT \
           (PIO_INPUT | PIO_CFG_DEGLITCH | \
            PIO_INT_BOTHEDGES | PIO_PORT_PIOB | PIO_PIN0 )
          
#define IRQ_FUSB302_INT SAM_IRQ_PB0
#endif

#define PIO_USBA_VBUS_SENSE \
            (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
            PIO_INT_BOTHEDGES | PIO_PORT_PIOA | PIO_PIN29 )
#define IRQ_USBA_VBUS_SENSE SAM_IRQ_PA29

#define PIO_USBB_VBUS_OVERCURRENT \
            (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
            PIO_INT_BOTHEDGES | PIO_PORT_PIOA | PIO_PIN28 )
#define IRQ_USBB_VBUS_OVERCURRENT SAM_IRQ_PA28

#define PIO_USBB_VBUS_ENABLE \
            (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_SET | \
            PIO_PORT_PIOC | PIO_PIN18)

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

#ifdef HAVE_AUDIO_CLASSD
int sam_audio_classd_initialize(int minor);
#endif /* HAVE_AUDIO_CLASSD */

void sam_spidev_initialize(void);
void sam_flexcom_spidev_initialize(void);

#ifdef HAVE_FUSB302
//int sam_fusb302init(FAR struct i2c_master_s *i2c, uint8_t fusb302addr);
int sam_fusb302init(int busno);
#endif

#ifdef HAVE_AT25
# define AT25_MINOR 0
int sam_at25_automount(int minor);
#endif



#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAMA5_JTI_TOUCAN2_SRC_JTI_TOUCAN2_H */
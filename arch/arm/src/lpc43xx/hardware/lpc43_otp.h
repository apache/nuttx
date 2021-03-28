/****************************************************************************
 * arch/arm/src/lpc43xx/hardware/lpc43_otp.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_OTP_H
#define __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_OTP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define LPC43_OTP_MEM00_OFFSET   0x0010  /* General purpose OTP memory 0, word 0 */
#define LPC43_OTP_MEM01_OFFSET   0x0014  /* General purpose OTP memory 0, word 1 */
#define LPC43_OTP_MEM02_OFFSET   0x0018  /* General purpose OTP memory 0, word 2 */
#define LPC43_OTP_MEM03_OFFSET   0x001c  /* General purpose OTP memory 0, word 3 */

#define LPC43_OTP_MEM10_OFFSET   0x0020  /* General purpose OTP memory 1, word 0 */
#define LPC43_OTP_MEM11_OFFSET   0x0024  /* General purpose OTP memory 1, word 1 */
#define LPC43_OTP_MEM12_OFFSET   0x0028  /* General purpose OTP memory 1, word 2 */
#define LPC43_OTP_MEM13_OFFSET   0x002c  /* General purpose OTP memory 1, word 3 */

#define LPC43_OTP_MEM20_OFFSET   0x0034  /* General purpose OTP memory 2, word 0 */
#define LPC43_OTP_MEM21_OFFSET   0x0038  /* General purpose OTP memory 2, word 1 */
#define LPC43_OTP_MEM22_OFFSET   0x003c  /* General purpose OTP memory 2, word 2 */

#define LPC43_OTP_AES00_OFFSET   0x0010  /* AES key 0, word 0 */
#define LPC43_OTP_AES01_OFFSET   0x0014  /* AES key 0, word 1 */
#define LPC43_OTP_AES02_OFFSET   0x0018  /* AES key 0, word 2 */
#define LPC43_OTP_AES03_OFFSET   0x001c  /* AES key 0, word 3 */

#define LPC43_OTP_AES10_OFFSET   0x0020  /* AES key 1, word 0 */
#define LPC43_OTP_AES11_OFFSET   0x0024  /* AES key 1, word 1 */
#define LPC43_OTP_AES12_OFFSET   0x0028  /* AES key 1, word 2 */
#define LPC43_OTP_AES13_OFFSET   0x002c  /* AES key 1, word 3 */

#define LPC43_OTP_CCD_OFFSET     0x0030  /* Customer control data */
#define LPC43_OTP_USBID_OFFSET   0x0034  /* USB ID */

/* Register Addresses *******************************************************/

#define LPC43_OTP_MEM00          (LPC43_OTPC_BASE+LPC43_OTP_MEM00_OFFSET)
#define LPC43_OTP_MEM01          (LPC43_OTPC_BASE+LPC43_OTP_MEM01_OFFSET)
#define LPC43_OTP_MEM02          (LPC43_OTPC_BASE+LPC43_OTP_MEM02_OFFSET)
#define LPC43_OTP_MEM03          (LPC43_OTPC_BASE+LPC43_OTP_MEM03_OFFSET)

#define LPC43_OTP_MEM10          (LPC43_OTPC_BASE+LPC43_OTP_MEM10_OFFSET)
#define LPC43_OTP_MEM11          (LPC43_OTPC_BASE+LPC43_OTP_MEM11_OFFSET)
#define LPC43_OTP_MEM12          (LPC43_OTPC_BASE+LPC43_OTP_MEM12_OFFSET)
#define LPC43_OTP_MEM13          (LPC43_OTPC_BASE+LPC43_OTP_MEM13_OFFSET)

#define LPC43_OTP_MEM20          (LPC43_OTPC_BASE+LPC43_OTP_MEM20_OFFSET)
#define LPC43_OTP_MEM21          (LPC43_OTPC_BASE+LPC43_OTP_MEM21_OFFSET)
#define LPC43_OTP_MEM22          (LPC43_OTPC_BASE+LPC43_OTP_MEM22_OFFSET)

#define LPC43_OTP_AES00          (LPC43_OTPC_BASE+LPC43_OTP_AES00_OFFSET)
#define LPC43_OTP_AES01          (LPC43_OTPC_BASE+LPC43_OTP_AES01_OFFSET)
#define LPC43_OTP_AES02          (LPC43_OTPC_BASE+LPC43_OTP_AES02_OFFSET)
#define LPC43_OTP_AES03          (LPC43_OTPC_BASE+LPC43_OTP_AES03_OFFSET)

#define LPC43_OTP_AES10          (LPC43_OTPC_BASE+LPC43_OTP_AES10_OFFSET)
#define LPC43_OTP_AES11          (LPC43_OTPC_BASE+LPC43_OTP_AES11_OFFSET)
#define LPC43_OTP_AES12          (LPC43_OTPC_BASE+LPC43_OTP_AES12_OFFSET)
#define LPC43_OTP_AES13          (LPC43_OTPC_BASE+LPC43_OTP_AES13_OFFSET)

#define LPC43_OTP_CCD            (LPC43_OTPC_BASE+LPC43_OTP_CCD_OFFSET)
#define LPC43_OTP_USBID          (LPC43_OTPC_BASE+LPC43_OTP_USBID_OFFSET)

/* Register Bit Definitions *************************************************/

/* Customer control data */

                                           /* Bits 0-22: Reserved */
#define OTP_CCD_USBID            (1 << 23) /* Bit 23: USB ID enable */
                                           /* Bit 24: Reserved */
#define OPT_CCD_BOOTSRC_SHIFT    (25)      /* Bits 25-28: Boot source selection in OTP */
#define OPT_CCD_BOOTSRC_MASK     (15 << OPT_CCD_BOOTSRC_SHIFT)
#  define OPT_CCD_BOOTSRC_EXT    (0 << OPT_CCD_BOOTSRC_SHIFT) /* External pins */
#  define OPT_CCD_BOOTSRC_USART0 (1 << OPT_CCD_BOOTSRC_SHIFT) /* USART0 */
#  define OPT_CCD_BOOTSRC_EMC8   (3 << OPT_CCD_BOOTSRC_SHIFT) /* EMC 8-bit */
#  define OPT_CCD_BOOTSRC_EMC16  (4 << OPT_CCD_BOOTSRC_SHIFT) /* EMC 16-bit */
#  define OPT_CCD_BOOTSRC_EMC32  (5 << OPT_CCD_BOOTSRC_SHIFT) /* EMC 32-bit */
#  define OPT_CCD_BOOTSRC_USB0   (6 << OPT_CCD_BOOTSRC_SHIFT) /* USB0 */
#  define OPT_CCD_BOOTSRC_USB1   (7 << OPT_CCD_BOOTSRC_SHIFT) /* USB1 */
#  define OPT_CCD_BOOTSRC_SPI    (8 << OPT_CCD_BOOTSRC_SHIFT) /* SPI (via SSP) */
#  define OPT_CCD_BOOTSRC_USART3 (9 << OPT_CCD_BOOTSRC_SHIFT) /* USART3 */

                                           /* Bits 29-30: Reserved */
#define OTP_CCD_JTAGDIS          (1 << 31) /* Bit 31: JTAG disable */

/* USB ID */

#define OTP_USBID_VID_SHIFT      (0)       /* Bits 0-15: USB vendor ID */
#define OTP_USBID_VID_MASK       (0xffff << OTP_USBID_VID_SHIFT)
#define OTP_USBID_PID_SHIFT      (0)       /* Bits 16-31: USB product ID */
#define OTP_USBID_PID_MASK       (0xffff << OTP_USBID_PID_SHIFT)

/* OTP API ******************************************************************/

/* The AES is controlled through a set of simple API calls located in the
 * LPC43xx ROM.  This value holds the pointer to the OTP driver table.
 */

#define LPC43_ROM_OTP_DRIVER_TABLE LPC43_ROM_DRIVER_TABLE1

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct lpc43_otp_s
{
  /* Initializes the OTP controller */

  unsigned int (*otp_Init)(void);

  /* Programs boot source */

  unsigned int (*otp_ProgBootSrc)(unsigned int src);

  /* JTAG disable.
   * This command disables JTAG only when the device is AES capable.
   */

  unsigned int (*otp_ProgJTAGDis)(void);

  /* Programs USB_ID */

  unsigned int (*otp_ProgUSBID)(unsigned int pid, unsigned int vid);

  /* Reserved */

  void *reserved[3];

  /* Program the general purpose OTP memories.
   * Use only if the device is not AES capable.
   */

  unsigned int (*otp_ProgGP0)(unsigned int data, unsigned int mask);
  unsigned int (*otp_ProgGP1)(unsigned int data, unsigned int mask);
  unsigned int (*otp_ProgGP2)(unsigned int data, unsigned int mask);

  /* Program AES keys.  16 byte keys are expected. */

  unsigned int (*otp_ProgKey1)(unsigned char *key);
  unsigned int (*otp_ProgKey2)(unsigned char *key);

  /* Generate new random number using the hardware
   * Random Number Generator (RNG).
   */

  unsigned int (*otp_GenRand)(void);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_OTP_H */

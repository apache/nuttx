/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_twi.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_TWI_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_TWI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf53_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets for TWI master (TWIM) ***********************************/

#define NRF53_TWIM_TASKS_STARTRX_OFFSET     0x0000 /* Start TWIM receive sequence */
#define NRF53_TWIM_TASKS_STARTTX_OFFSET     0x0008 /* Start TWIM transmit sequence */
#define NRF53_TWIM_TASKS_STOP_OFFSET        0x0014 /* Stop TWIM transaction */
#define NRF53_TWIM_TASKS_SUSPEND_OFFSET     0x001c /* Suspend TWIM transaction */
#define NRF53_TWIM_TASKS_RESUME_OFFSET      0x0020 /* Resume TWIM transaction */
                                                   /* TODO: 0x080 - 0x0a0 */
#define NRF53_TWIM_EVENTS_STOPPED_OFFSET    0x0104 /* TWIM stopped */
#define NRF53_TWIM_EVENTS_ERROR_OFFSET      0x0124 /* TWIM error */
#define NRF53_TWIM_EVENTS_SUSPENDED_OFFSET  0x0148 /* Last byte has been sent out after the SUSPEND task has been issued */
#define NRF53_TWIM_EVENTS_RXSTARTED_OFFSET  0x014c /* Receive sequence started */
#define NRF53_TWIM_EVENTS_TXSTARTED_OFFSET  0x0150 /* Transmit sequence started */
#define NRF53_TWIM_EVENTS_LASTRX_OFFSET     0x015c /* Byte boundary, starting to receive the last byte */
#define NRF53_TWIM_EVENTS_LASTTX_OFFSET     0x0160 /* Byte boundary, starting to transmit the last byte */
                                                   /* TODO: 0x184 - 0x1e0 */
#define NRF53_TWIM_SHORTS_OFFSET            0x0200 /* Shortcuts between local events and tasks */
#define NRF53_TWIM_INTEN_OFFSET             0x0300 /* Enable or disable interrupt */
#define NRF53_TWIM_INTENSET_OFFSET          0x0304 /* Enable interrupt */
#define NRF53_TWIM_INTENCLR_OFFSET          0x0308 /* Disable interrupt */
#define NRF53_TWIM_ERRORSRC_OFFSET          0x04c4 /* Error source */
#define NRF53_TWIM_ENABLE_OFFSET            0x0500 /* Enable TWIMS */
#define NRF53_TWIM_PSELSCL_OFFSET           0x0508 /* Pin select for SCL signal */
#define NRF53_TWIM_PSELSDA_OFFSET           0x050c /* Pin select for SDA signal */
#define NRF53_TWIM_FREQUENCY_OFFSET         0x0524 /* TWIM frequency */
#define NRF53_TWIM_RXDPTR_OFFSET            0x0534 /* RXD Data pointer */
#define NRF53_TWIM_RXDMAXCNT_OFFSET         0x0538 /* Maximum number of bytes in RXD buffer */
#define NRF53_TWIM_RXDAMOUNT_OFFSET         0x053c /* Number of bytes transferred in the last RXD transaction */
#define NRF53_TWIM_RXDLIST_OFFSET           0x0540 /* RX EasyDMA list type */
#define NRF53_TWIM_TXDPTR_OFFSET            0x0544 /* TXD Data pointer */
#define NRF53_TWIM_TXMAXCNT_OFFSET          0x0548 /* Maximum number of bytes in TXD buffer */
#define NRF53_TWIM_TXAMOUNT_OFFSET          0x054c /* Number of bytes transferred in the last TXD transaction */
#define NRF53_TWIM_TXLIST_OFFSET            0x0550 /* TX EasyDMA list type */
#define NRF53_TWIM_ADDRESS_OFFSET           0x0588 /* TWIM address */

/* Register offsets for TWI slave (TWIS) ************************************/

#define NRF53_TWIS_TASKS_STOP_OFFSET        0x0014 /* Stop TWIS transaction */
#define NRF53_TWIS_TASKS_SUSPEND_OFFSET     0x001c /* Suspend TWIS transaction */
#define NRF53_TWIS_TASKS_RESUME_OFFSET      0x0020 /* Resume TWIS transaction */
#define NRF53_TWIS_TASKS_PREPARERX_OFFSET   0x0030 /* Prepare the TWIS slave to respond to a write command */
#define NRF53_TWIS_TASKS_PREPARETX_OFFSET   0x0034 /* Prepare the TWIS slave to respond to a read command */
#define NRF53_TWIS_EVENTS_STOPPED_OFFSET    0x0104 /* TWIS stopped */
#define NRF53_TWIS_EVENTS_ERROR_OFFSET      0x0124 /* TWIS error */
#define NRF53_TWIS_EVENTS_RXSTARTED_OFFSET  0x014c /* Receive sequence started */
#define NRF53_TWIS_EVENTS_TXSTARTED_OFFSET  0x0150 /* Transmit sequence started */
#define NRF53_TWIS_EVENTS_WRITE_OFFSET      0x0164 /* Write command received */
#define NRF53_TWIS_EVENTS_READ_OFFSET       0x0168 /* Read command received */
#define NRF53_TWIS_SHORTS_OFFSET            0x0200 /* Shortcuts between local events and tasks */
#define NRF53_TWIS_INTEN_OFFSET             0x0300 /* Enable or disable interrupt */
#define NRF53_TWIS_INTENSET_OFFSET          0x0304 /* Enable interrupt */
#define NRF53_TWIS_INTENCLR_OFFSET          0x0308 /* Disable interrupt */
#define NRF53_TWIS_ERRORSRC_OFFSET          0x04d0 /* Error source */
#define NRF53_TWIS_MATCH_OFFSET             0x04d4 /* Status register indicating which address had a match */
#define NRF53_TWIS_ENABLE_OFFSET            0x0500 /* Enable TWISS */
#define NRF53_TWIS_PSELSCL_OFFSET           0x0508 /* Pin select for SCL signal */
#define NRF53_TWIS_PSELSDA_OFFSET           0x050c /* Pin select for SDA signal */
#define NRF53_TWIS_RXDPTR_OFFSET            0x0534 /* RXD Data pointer */
#define NRF53_TWIS_RXDMAXCNT_OFFSET         0x0538 /* Maximum number of bytes in RXD buffer */
#define NRF53_TWIS_RXDAMOUNT_OFFSET         0x053c /* Number of bytes transferred in the last RXD transaction */
#define NRF53_TWIS_RXDLIST_OFFSET           0x0540 /* RX EasyDMA list type */
#define NRF53_TWIS_TXDPTR_OFFSET            0x0544 /* TXD Data pointer */
#define NRF53_TWIS_TXMAXCNT_OFFSET          0x0548 /* Maximum number of bytes in TXD buffer */
#define NRF53_TWIS_TXAMOUNT_OFFSET          0x054c /* Number of bytes transferred in the last TXD transaction */
#define NRF53_TWIS_TXLIST_OFFSET            0x0550 /* TX EasyDMA list type */
#define NRF53_TWIS_ADDRESS0_OFFSET          0x0588 /* TWIS address 0 */
#define NRF53_TWIS_ADDRESS1_OFFSET          0x058c /* TWIS address 1 */
#define NRF53_TWIS_CONFIG_OFFSET            0x0594 /* Configuration register for the address match mechanism */
#define NRF53_TWIS_ORC_OFFSET               0x05c0 /* Over-read character */

/* Register Bitfield Definitions for TWIM ***********************************/

/* SHORTS Register */

#define TWIM_SHORTS_LASTTX_STARTRX          (1 << 7)   /* Bit 7: Shortcut between event LASTTX and task STARTRX */
#define TWIM_SHORTS_LASTTX_SUSPEND          (1 << 8)   /* Bit 8: Shortcut between event LASTTX and task SUSPEND */
#define TWIM_SHORTS_LASTTX_STOP             (1 << 9)   /* Bit 9: Shortcut between event LASTTX and task STOP */
#define TWIM_SHORTS_LASTRX_STARTTX          (1 << 10)  /* Bit 10: Shortcut between event LASTRX and task STARTTX */
#define TWIM_SHORTS_LASTRX_SUSPEND          (1 << 11)  /* Bit 11: Shortcut between event LASTRX and task SUSPEND */
#define TWIM_SHORTS_LASTRX_STOP             (1 << 12)  /* Bit 12: Shortcut between event LASTRX and task STOP */

/* INTEN/INTENSET/INTENCLR Register */

#define TWIM_INT_STOPPED                    (1 << 1)   /* Bit 1: Interrupt for event STOPPED */
#define TWIM_INT_ERROR                      (1 << 9)   /* Bit 9: Interrupt for event ERROR */
#define TWIM_INT_SUSPENDED                  (1 << 18)  /* Bit 18: Interrupt for event SUSPENDED */
#define TWIM_INT_RXSTARTED                  (1 << 19)  /* Bit 19: Interrupt for event RXSTARTED */
#define TWIM_INT_TXSTARTED                  (1 << 20)  /* Bit 20: Interrupt for event TXSTARTED */
#define TWIM_INT_LASTRX                     (1 << 23)  /* Bit 23: Interrupt for event LASTRX */
#define TWIM_INT_LASTTX                     (1 << 24)  /* Bit 24: Interrupt for event LASTTX */

/* ERRORSRC Register */

#define TWIM_ERRORSRC_OVERRUN               (1 << 0)   /* Bit 0: Overrun error */
#define TWIM_ERRORSRC_ANACK                 (1 << 1)   /* Bit 1: NACK received after sending the address */
#define TWIM_ERRORSRC_DNACK                 (1 << 2)   /* Bit 2: NACK received after sending a data byte */

/* ENABLE Register */

#define TWIM_ENABLE_DIS                     (0)        /* Disable TWIM */
#define TWIM_ENABLE_EN                      (0x6 << 0) /* Disable TWIM */

/* PSELSCL Register */

#define TWIM_PSELSCL_PIN_SHIFT              (0)        /* Bits 0-4: SCL pin number */
#define TWIM_PSELSCL_PIN_MASK               (0x1f << TWIM_PSELSCL_PIN_SHIFT)
#define TWIM_PSELSCL_PORT_SHIFT             (5)        /* Bit 5: SCL port number */
#define TWIM_PSELSCL_PORT_MASK              (0x1 << TWIM_PSELSCL_PORT_SHIFT)
#define TWIM_PSELSCL_CONNECTED              (1 << 31)  /* Bit 31: Connection */
#define TWIM_PSELSCL_RESET                  (0xffffffff)

/* PSELSDA Register */

#define TWIM_PSELSDA_PIN_SHIFT              (0)        /* Bits 0-4: SDA pin number */
#define TWIM_PSELSDA_PIN_MASK               (0x1f << TWIM_PSELSDA_PIN_SHIFT)
#define TWIM_PSELSDA_PORT_SHIFT             (5)        /* Bit 5: SDA port number */
#define TWIM_PSELSDA_PORT_MASK              (0x1 << TWIM_PSELSDA_PORT_SHIFT)
#define TWIM_PSELSDA_CONNECTED              (1 << 31)  /* Bit 31: Connection */
#define TWIM_PSELSDA_RESET                  (0xffffffff)

/* FREQUENCY Register */

#define TWIM_FREQUENCY_100KBPS              (0x01980000) /* 100 kbps */
#define TWIM_FREQUENCY_250KBPS              (0x04000000) /* 250 kbps */
#define TWIM_FREQUENCY_400KBPS              (0x06400000) /* 400 kbps */
#define TWIM_FREQUENCY_1000KBPS             (0x0ff00000) /* 1000 kbps */

/* RXDMAXCNT Register */

#define TWIM_RXDMAXCNT_SHIFT                (0)        /* Bits 0-15: Maximum number of bytes in receive buffer */
#define TWIM_RXDMAXCNT_MASK                 (0xffff << TWIM_RXDMAXCNT_SHIFT)

/* RXDAMOUNT Register */

#define TWIM_RXDAMOUNT_SHIFT                (0)        /* Bits 0-15: Number of bytes transferred in the last transaction */
#define TWIM_RXDAMOUNT_MASK                 (0xffff << TWIM_RXDAMOUNT_SHIFT)

/* TXDMAXCNT Register */

#define TWIM_TXDMAXCNT_SHIFT                (0)        /* Bits 0-15: Maximum number of bytes in transmit buffer */
#define TWIM_TXDMAXCNT_MASK                 (0xffff << TWIM_TXDMAXCNT_SHIFT)

/* TXDAMOUNT Register */

#define TWIM_TXDAMOUNT_SHIFT                (0)        /* Bits 0-15: Number of bytes transferred in the last transaction */
#define TWIM_TXDAMOUNT_MASK                 (0xffff << TWIM_TXDAMOUNT_SHIFT)

/* ADDRESS Register */

#define TWIM_ADDRESS_SHIFT                  (0)        /* Bits 0-6: Address used in the TWI transfer */
#define TWIM_ADDRESS_MASK                   (0x7f << TWIM_ADDRESS_SHIFT)

/* Register Bitfield Definitions for TWIS ***********************************/

/* TODO */

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_TWI_H */

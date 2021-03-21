/****************************************************************************
 * boards/z80/ez80/ez80f910200zco/src/ez80f910200zco.h
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

#ifndef __BOARDS_Z80_EZ80_EZ80F910200ZCO_SRC_EZ80F910200ZCO_H
#define __BOARDS_Z80_EZ80_EZ80F910200ZCO_SRC_EZ80F910200ZCO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory map.  Board-specific extensions to the basic ez80f91 memory map
 * (see arch/z80/src/ez80/ez80f91.h
 */

                                   /* CS0: 0x000000 256Kb of on-chip flash */
#define EZ80_OFFCHIPFLASH 0x400000 /* CS0: Off chip flash (Up to 4Mb-256Kb) */
#define EZ80_LEDGPIOCNTRL 0x800000 /* CS2: (See below) */
#define EZ80_PLTFMSRAM    0xb80000 /* CS2: Platform SRAM (512Kb) */
#define EZ80_MODULESRAM   0xc00000 /* CS1: Module SRAM (up to 2Mb) */
                                   /* 0xffc000 On-chip EMAC SRAM (8Kb) */
                                   /* 0xffe000 On-chip SRAM (8Kb) */

/* LED and port emulation memory register addresses */

#define EZ80_LEDANODE     0x800000 /* WR: LED anode/GPIO port output control */
#define EZ80_GPIOCNTRL    EZ80_LEDANODE
#define EZ80_LEDCATHODE   0x800001 /* WR: LED cathode/Modem/Trig */
#define EZ80_MODEM        EZ80_LEDCATHODE
#define EZ80_TRIGGERS     EZ80_LEDCATHODE
#define EZ80_GPIODATA     0x800002 /* RD/WR: GPIO data */

#define ez80_getmmreg8(a)   (*(uint8_t*)(a))
#define ez80_putmmreg8(v,a) (*(uint8_t*)(a) = (v))

/* LED anode/GPIO port output control bit definitions */

#define EZ80_ANODECOL1    0x01
#define EZ80_ANODECOL2    0x02
#define EZ80_ANODECOL3    0x04
#define EZ80_ANODECOL4    0x08
#define EZ80_ANODECOL5    0x10
#define EZ80_ANODECOL6    0x20
#define EZ80_ANODECOL7    0x40
#define EZ80_GPIOOUTPUT   0x80

/* LED cathode/Modem/Trig bit definitions */

#define EZ80_CATHODEROW5  0x01
#define EZ80_CATHODEROW4  0x02
#define EZ80_CATHODEROW3  0x04
#define EZ80_CATHODEROW2  0x08
#define EZ80_CATHODEROW1  0x10
#define EZ80_MODEMRESET   0x20
#define EZ80_TRIG1        0x40
#define EZ80_TRIG2        0x80

/* GPIO data bit definitions */

#define EZ80_GPIOD0       0x01
#define EZ80_GPIOD1       0x02
#define EZ80_GPIOD2       0x04
#define EZ80_GPIOD3       0x08
#define EZ80_GPIOD4       0x10
#define EZ80_GPIOD5       0x20
#define EZ80_GPIOD6       0x40
#define EZ80_GPIOD7       0x80

/* Modem Signals:
 *
 * DCD:
 *   The Data Carrier Detect (DCD) signal at D1 indicates that a good carrier
 *   signal is being received from the remove mode.
 * RX:
 *   The RX signal at D2 indicates that data is received from the modem.
 * DTR:
 *   The Data Terminal Ready (DTR) signal at D3 informs the modem that the PC
 *   is ready.
 * TX:
 *   The TX signal at D4 indicates that data is transmitted to the modem.
 */

/* Push buttons:
 *
 * PB0   SW1 Bit 0 of GPIO Port B
 * PB1   SW2 Bit 1 of GPIO Port B
 * PB2   SW3 Bit 2 of GPIO Port B
 * RESET SW4
 */

#define EZ80_PB0_IRQ EZ80_PORTB0_IRQ  /* Vector Oxa0 */
#define EZ80_PB1_IRQ EZ80_PORTB1_IRQ  /* Vector Oxa4 */
#define EZ80_PB2_IRQ EZ80_PORTB2_IRQ  /* Vector Oxa8 */

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __BOARDS_Z80_EZ80_EZ80F910200ZCO_SRC_EZ80F910200ZCO_H */

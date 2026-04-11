/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_qdec.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_QDEC_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_QDEC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf52_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF52_QDEC_TASKS_START_OFFSET       0x0000  /* Task starting the quadrature decoder */
#define NRF52_QDEC_TASKS_STOP_OFFSET        0x0004  /* Task stopping the quadrature decoder */
#define NRF52_QDEC_TASKS_READCLRACC_OFFSET  0x0008  /* Read and clear ACC and ACCDBL */
#define NRF52_QDEC_TASKS_RDCLRACC_OFFSET    0x000c  /* Read and clear ACC */
#define NRF52_QDEC_TASKS_RDCLRDBL_OFFSET    0x0010  /* Read and clear ACCDBL */

#define NRF52_QDEC_EVENTS_SAMPLERDY_OFFSET  0x0100  /* Event being generated for every new sample value */
#define NRF52_QDEC_EVENTS_REPORTRDY_OFFSET  0x0104  /* Non-null report ready */
#define NRF52_QDEC_EVENTS_ACCOF_OFFSET      0x0108  /* ACC or ACCDBL register overflow */
#define NRF52_QDEC_EVENTS_DBLRDY_OFFSET     0x010c  /* Double displacement(s) detected */
#define NRF52_QDEC_EVENTS_STOPPED_OFFSET    0x0110  /* QDEC has been stopped */

#define NRF52_QDEC_SHORTS_OFFSET            0x0200  /* Shortcut register */
#define NRF52_QDEC_INTENSET_OFFSET          0x0304  /* Enable interrupt */
#define NRF52_QDEC_INTENCLR_OFFSET          0x0308  /* Disable interrupt */
#define NRF52_QDEC_ENABLE_OFFSET            0x0500  /* Enable the quadrature decoder */
#define NRF52_QDEC_LEDPOL_OFFSET            0x0504  /* LED output pin polarity */
#define NRF52_QDEC_SAMPLEPER_OFFSET         0x0508  /* Sample period */
#define NRF52_QDEC_SAMPLE_OFFSET            0x050c  /* Motion sample value */
#define NRF52_QDEC_REPORTPER_OFFSET         0x0510  /* Number of samples to be taken before REPORTRDY and DBLRDY events */
#define NRF52_QDEC_ACC_OFFSET               0x0514  /* Register accumulating the valid transitions */
#define NRF52_QDEC_ACCREAD_OFFSET           0x0518  /* Snapshot of the ACC register */
#define NRF52_QDEC_PSEL_LED_OFFSET          0x051c  /* Pin select for LED signal */
#define NRF52_QDEC_PSEL_A_OFFSET            0x0520  /* Pin select for A signal */
#define NRF52_QDEC_PSEL_B_OFFSET            0x0524  /* Pin select for B signal */
#define NRF52_QDEC_DBFEN_OFFSET             0x0528  /* Enable input debounce filters */
#define NRF52_QDEC_LEDPRE_OFFSET            0x0540  /* Time period the LED is switched ON prior to sampling */
#define NRF52_QDEC_ACCDBL_OFFSET            0x0544  /* Register accumulating the number of detected double transitions */
#define NRF52_QDEC_ACCDBLREAD_OFFSET        0x0548  /* Snapshot of the ACCDBL register */

/* Register addresses *******************************************************/

#define NRF52_QDEC_TASKS_START              (NRF52_QDEC_BASE + NRF52_QDEC_TASKS_START_OFFSET)
#define NRF52_QDEC_TASKS_STOP               (NRF52_QDEC_BASE + NRF52_QDEC_TASKS_STOP_OFFSET)
#define NRF52_QDEC_TASKS_READCLRACC         (NRF52_QDEC_BASE + NRF52_QDEC_TASKS_READCLRACC_OFFSET)
#define NRF52_QDEC_TASKS_RDCLRACC           (NRF52_QDEC_BASE + NRF52_QDEC_TASKS_RDCLRACC_OFFSET)
#define NRF52_QDEC_TASKS_RDCLRDBL           (NRF52_QDEC_BASE + NRF52_QDEC_TASKS_RDCLRDBL_OFFSET)

#define NRF52_QDEC_EVENTS_SAMPLERDY         (NRF52_QDEC_BASE + NRF52_QDEC_EVENTS_SAMPLERDY_OFFSET)
#define NRF52_QDEC_EVENTS_REPORTRDY         (NRF52_QDEC_BASE + NRF52_QDEC_EVENTS_REPORTRDY_OFFSET)
#define NRF52_QDEC_EVENTS_ACCOF             (NRF52_QDEC_BASE + NRF52_QDEC_EVENTS_ACCOF_OFFSET)
#define NRF52_QDEC_EVENTS_DBLRDY            (NRF52_QDEC_BASE + NRF52_QDEC_EVENTS_DBLRDY_OFFSET)
#define NRF52_QDEC_EVENTS_STOPPED           (NRF52_QDEC_BASE + NRF52_QDEC_EVENTS_STOPPED_OFFSET)

#define NRF52_QDEC_SHORTS                   (NRF52_QDEC_BASE + NRF52_QDEC_SHORTS_OFFSET)
#define NRF52_QDEC_INTENSET                 (NRF52_QDEC_BASE + NRF52_QDEC_INTENSET_OFFSET)
#define NRF52_QDEC_INTENCLR                 (NRF52_QDEC_BASE + NRF52_QDEC_INTENCLR_OFFSET)
#define NRF52_QDEC_ENABLE                   (NRF52_QDEC_BASE + NRF52_QDEC_ENABLE_OFFSET)
#define NRF52_QDEC_LEDPOL                   (NRF52_QDEC_BASE + NRF52_QDEC_LEDPOL_OFFSET)
#define NRF52_QDEC_SAMPLEPER                (NRF52_QDEC_BASE + NRF52_QDEC_SAMPLEPER_OFFSET)
#define NRF52_QDEC_SAMPLE                   (NRF52_QDEC_BASE + NRF52_QDEC_SAMPLE_OFFSET)
#define NRF52_QDEC_REPORTPER                (NRF52_QDEC_BASE + NRF52_QDEC_REPORTPER_OFFSET)
#define NRF52_QDEC_ACC                      (NRF52_QDEC_BASE + NRF52_QDEC_ACC_OFFSET)
#define NRF52_QDEC_ACCREAD                  (NRF52_QDEC_BASE + NRF52_QDEC_ACCREAD_OFFSET)
#define NRF52_QDEC_PSEL_LED                 (NRF52_QDEC_BASE + NRF52_QDEC_PSEL_LED_OFFSET)
#define NRF52_QDEC_PSEL_A                   (NRF52_QDEC_BASE + NRF52_QDEC_PSEL_A_OFFSET)
#define NRF52_QDEC_PSEL_B                   (NRF52_QDEC_BASE + NRF52_QDEC_PSEL_B_OFFSET)
#define NRF52_QDEC_DBFEN                    (NRF52_QDEC_BASE + NRF52_QDEC_DBFEN_OFFSET)
#define NRF52_QDEC_LEDPRE                   (NRF52_QDEC_BASE + NRF52_QDEC_LEDPRE_OFFSET)
#define NRF52_QDEC_ACCDBL                   (NRF52_QDEC_BASE + NRF52_QDEC_ACCDBL_OFFSET)
#define NRF52_QDEC_ACCDBLREAD               (NRF52_QDEC_BASE + NRF52_QDEC_ACCDBLREAD_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* TASKS_START Register */

#define QDEC_TASKS_START                    (1 << 0)  /* Bit 0: Start QDEC */

/* TASKS_STOP Register */

#define QDEC_TASKS_STOP                     (1 << 0)  /* Bit 0: Stop QDEC */

/* TASKS_READCLRACC Register */

#define QDEC_TASKS_READCLRACC               (1 << 0)  /* Bit 0: Read and clear ACC and ACCDBL */

/* TASKS_RDCLRACC Register */

#define QDEC_TASKS_RDCLRACC                 (1 << 0)  /* Bit 0: Read and clear ACC */

/* TASKS_RDCLRDBL Register */

#define QDEC_TASKS_RDCLRDBL                 (1 << 0)  /* Bit 0: Read and clear ACCDBL */

/* EVENTS_SAMPLERDY Register */

#define QDEC_EVENTS_SAMPLERDY               (1 << 0)  /* Bit 0: Sample ready event */

/* EVENTS_REPORTRDY Register */

#define QDEC_EVENTS_REPORTRDY               (1 << 0)  /* Bit 0: Report ready event */

/* EVENTS_ACCOF Register */

#define QDEC_EVENTS_ACCOF                   (1 << 0)  /* Bit 0: Accumulator overflow event */

/* EVENTS_DBLRDY Register */

#define QDEC_EVENTS_DBLRDY                  (1 << 0)  /* Bit 0: Double displacement ready event */

/* EVENTS_STOPPED Register */

#define QDEC_EVENTS_STOPPED                 (1 << 0)  /* Bit 0: QDEC stopped event */

/* SHORTS Register */

#define QDEC_SHORTS_REPORTRDY_READCLRACC    (1 << 0)  /* Bit 0: Shortcut between REPORTRDY event and READCLRACC task */
#define QDEC_SHORTS_SAMPLERDY_STOP          (1 << 1)  /* Bit 1: Shortcut between SAMPLERDY event and STOP task */
#define QDEC_SHORTS_REPORTRDY_RDCLRACC      (1 << 2)  /* Bit 2: Shortcut between REPORTRDY event and RDCLRACC task */
#define QDEC_SHORTS_REPORTRDY_STOP          (1 << 3)  /* Bit 3: Shortcut between REPORTRDY event and STOP task */
#define QDEC_SHORTS_DBLRDY_RDCLRDBL         (1 << 4)  /* Bit 4: Shortcut between DBLRDY event and RDCLRDBL task */
#define QDEC_SHORTS_DBLRDY_STOP             (1 << 5)  /* Bit 5: Shortcut between DBLRDY event and STOP task */
#define QDEC_SHORTS_SAMPLERDY_READCLRACC    (1 << 6)  /* Bit 6: Shortcut between SAMPLERDY event and READCLRACC task */

/* INTENSET/INTENCLR Register */

#define QDEC_INT_SAMPLERDY                  (1 << 0)  /* Bit 0: Interrupt for SAMPLERDY event */
#define QDEC_INT_REPORTRDY                  (1 << 1)  /* Bit 1: Interrupt for REPORTRDY event */
#define QDEC_INT_ACCOF                      (1 << 2)  /* Bit 2: Interrupt for ACCOF event */
#define QDEC_INT_DBLRDY                     (1 << 3)  /* Bit 3: Interrupt for DBLRDY event */
#define QDEC_INT_STOPPED                    (1 << 4)  /* Bit 4: Interrupt for STOPPED event */

/* ENABLE Register */

#define QDEC_ENABLE_DISABLE                 (0 << 0)  /* Bit 0: Disable QDEC */
#define QDEC_ENABLE_ENABLE                  (1 << 0)  /* Bit 0: Enable QDEC */

/* LEDPOL Register */

#define QDEC_LEDPOL_ACTIVELOW               (0 << 0)  /* Bit 0: LED active on output pin low */
#define QDEC_LEDPOL_ACTIVEHIGH              (1 << 0)  /* Bit 0: LED active on output pin high */

/* SAMPLEPER Register */

#define QDEC_SAMPLEPER_SHIFT                (0)       /* Bits 0-3: Sample period */
#define QDEC_SAMPLEPER_MASK                 (0xf << QDEC_SAMPLEPER_SHIFT)
#  define QDEC_SAMPLEPER_128US              (0 << QDEC_SAMPLEPER_SHIFT)  /* 128 us */
#  define QDEC_SAMPLEPER_256US              (1 << QDEC_SAMPLEPER_SHIFT)  /* 256 us */
#  define QDEC_SAMPLEPER_512US              (2 << QDEC_SAMPLEPER_SHIFT)  /* 512 us */
#  define QDEC_SAMPLEPER_1024US             (3 << QDEC_SAMPLEPER_SHIFT)  /* 1024 us */
#  define QDEC_SAMPLEPER_2048US             (4 << QDEC_SAMPLEPER_SHIFT)  /* 2048 us */
#  define QDEC_SAMPLEPER_4096US             (5 << QDEC_SAMPLEPER_SHIFT)  /* 4096 us */
#  define QDEC_SAMPLEPER_8192US             (6 << QDEC_SAMPLEPER_SHIFT)  /* 8192 us */
#  define QDEC_SAMPLEPER_16384US            (7 << QDEC_SAMPLEPER_SHIFT)  /* 16384 us */
#  define QDEC_SAMPLEPER_32MS               (8 << QDEC_SAMPLEPER_SHIFT)  /* 32768 us */
#  define QDEC_SAMPLEPER_65MS               (9 << QDEC_SAMPLEPER_SHIFT)  /* 65536 us */
#  define QDEC_SAMPLEPER_131MS              (10 << QDEC_SAMPLEPER_SHIFT) /* 131072 us */

/* SAMPLE Register */

#define QDEC_SAMPLE_MASK                    (0xffffffff)

/* REPORTPER Register */

#define QDEC_REPORTPER_SHIFT                (0)       /* Bits 0-3: Report period */
#define QDEC_REPORTPER_MASK                 (0xf << QDEC_REPORTPER_SHIFT)
#  define QDEC_REPORTPER_10SMPL             (0 << QDEC_REPORTPER_SHIFT)  /* 10 samples */
#  define QDEC_REPORTPER_40SMPL             (1 << QDEC_REPORTPER_SHIFT)  /* 40 samples */
#  define QDEC_REPORTPER_80SMPL             (2 << QDEC_REPORTPER_SHIFT)  /* 80 samples */
#  define QDEC_REPORTPER_120SMPL            (3 << QDEC_REPORTPER_SHIFT)  /* 120 samples */
#  define QDEC_REPORTPER_160SMPL            (4 << QDEC_REPORTPER_SHIFT)  /* 160 samples */
#  define QDEC_REPORTPER_200SMPL            (5 << QDEC_REPORTPER_SHIFT)  /* 200 samples */
#  define QDEC_REPORTPER_240SMPL            (6 << QDEC_REPORTPER_SHIFT)  /* 240 samples */
#  define QDEC_REPORTPER_280SMPL            (7 << QDEC_REPORTPER_SHIFT)  /* 280 samples */
#  define QDEC_REPORTPER_DISABLED           (8 << QDEC_REPORTPER_SHIFT)  /* Disabled */

/* ACC Register */

#define QDEC_ACC_MASK                       (0xffffffff)

/* ACCREAD Register */

#define QDEC_ACCREAD_MASK                   (0xffffffff)

/* PSEL Registers */

#define QDEC_PSEL_PIN_SHIFT                 (0)       /* Bits 0-4: Pin number */
#define QDEC_PSEL_PIN_MASK                  (0x1f << QDEC_PSEL_PIN_SHIFT)
#define QDEC_PSEL_PORT_SHIFT                (5)       /* Bit 5: Port number */
#define QDEC_PSEL_PORT_MASK                 (0x1 << QDEC_PSEL_PORT_SHIFT)
#define QDEC_PSEL_CONNECT_SHIFT             (31)      /* Bit 31: Connection */
#define QDEC_PSEL_CONNECT_MASK              (0x1 << QDEC_PSEL_CONNECT_SHIFT)
#  define QDEC_PSEL_CONNECTED               (0 << QDEC_PSEL_CONNECT_SHIFT)
#  define QDEC_PSEL_DISCONNECTED            (1 << QDEC_PSEL_CONNECT_SHIFT)

/* DBFEN Register */

#define QDEC_DBFEN_DISABLE                  (0 << 0)  /* Bit 0: Debounce input filters disabled */
#define QDEC_DBFEN_ENABLE                   (1 << 0)  /* Bit 0: Debounce input filters enabled */

/* LEDPRE Register */

#define QDEC_LEDPRE_SHIFT                   (0)       /* Bits 0-8: LED pre time */
#define QDEC_LEDPRE_MASK                    (0x1ff << QDEC_LEDPRE_SHIFT)

/* ACCDBL Register */

#define QDEC_ACCDBL_MASK                    (0xf)

/* ACCDBLREAD Register */

#define QDEC_ACCDBLREAD_MASK                (0xf)

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_QDEC_H */

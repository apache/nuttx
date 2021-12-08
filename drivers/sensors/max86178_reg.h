/****************************************************************************
 * drivers/sensors/max86178_reg.h
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

#ifndef __DRIVERS_SENSORS_MAX86178_REG_H
#define __DRIVERS_SENSORS_MAX86178_REG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_SENSORS_MAX86178

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MAX86178 registers addresses */

#define MAX86178_REG_STAT1          0x00 /* Status 1 register */
#define MAX86178_REG_STAT2          0x01 /* Status 2 register */
#define MAX86178_REG_STAT3          0x02 /* Status 3 register */
#define MAX86178_REG_STAT4          0x03 /* Status 4 register */
#define MAX86178_REG_STAT5          0x04 /* Status 5 register */
#define MAX86178_REG_FIFOWRPT       0x08 /* FIFO write pointer */
#define MAX86178_REG_FIFORDPT       0x09 /* FIFO read pointer */
#define MAX86178_REG_FIFOCNT1       0x0a /* FIFO counter 1 */
#define MAX86178_REG_FIFOCNT2       0x0b /* FIFO counter 2 */
#define MAX86178_REG_FIFOCNTLSB     0x0b /* An alternative name of FIFOCNT2 */
#define MAX86178_REG_FIFODATA       0x0c /* FIFO data */
#define MAX86178_REG_FIFOCFG1       0x0d /* FIFO configuration 1 */
#define MAX86178_REG_FIFOAFULL      0x0d /* An alternative name of FIFOCFG1 */
#define MAX86178_REG_FIFOCFG2       0x0e /* FIFO configuration 2 */
#define MAX86178_REG_SYSSYNC        0x10 /* System sync */
#define MAX86178_REG_SYSCFG1        0x11 /* System configuration 1 */
#define MAX86178_REG_SYSCFG2        0x12 /* System configuration 2 */
#define MAX86178_REG_PINFUNC        0x13 /* Pin functional config */
#define MAX86178_REG_PINCFG         0x14 /* Output pin configuration */
#define MAX86178_REG_BCASTADDR      0x15 /* I2C broadcast address */
#define MAX86178_REG_PLLCFG1        0x18 /* PLL configuration 1 */
#define MAX86178_REG_PLLCFG2        0x19 /* PLL configuration 2 */
#define MAX86178_REG_MDIVLSB        0x19 /* An alternative name of PLLCFG2 */
#define MAX86178_REG_PLLCFG3        0x1a /* PLL configuration 3 */
#define MAX86178_REG_PLLCFG4        0x1b /* PLL configuration 4 */
#define MAX86178_REG_PLLCFG5        0x1c /* PLL configuration 5 */
#define MAX86178_REG_ECGNDIVLSB     0x1c /* An alternative name of PLLCFG5 */
#define MAX86178_REG_PLLCFG6        0x1d /* PLL configuration 6 */
#define MAX86178_REG_PPGCFG1        0x20 /* PPG configuration 1 */
#define MAX86178_REG_PPGCFG2        0x21 /* PPG configuration 2 */
#define MAX86178_REG_PPGCFG3        0x22 /* PPG configuration 3 */
#define MAX86178_REG_PPGCFG4        0x23 /* PPG configuration 4 */
#define MAX86178_REG_PDBIAS         0x24 /* Photodiode bias */
#define MAX86178_REG_FRCLKDIVH      0x28 /* Frame clock divider MSB */
#define MAX86178_REG_FRCLKDIVL      0x29 /* Frame clock divider LSB */
#define MAX86178_REG_MEAS1SEL       0x30 /* Measurement 1 selects */
#define MAX86178_REG_MEAS1CFG1      0x31 /* Measurement 1 configuration 1 */
#define MAX86178_REG_MEAS1CFG2      0x32 /* Measurement 1 configuration 2 */
#define MAX86178_REG_MEAS1CFG3      0x33 /* Measurement 1 configuration 3 */
#define MAX86178_REG_MEAS1CFG4      0x34 /* Measurement 1 configuration 4 */
#define MAX86178_REG_MEAS1CFG5      0x35 /* Measurement 1 configuration 5 */
#define MAX86178_REG_MEAS1LEDA      0x36 /* Measurement 1 LEDA current */
#define MAX86178_REG_MEAS1LEDB      0x37 /* Measurement 1 LEDB current */
#define MAX86178_REG_MEAS2SEL       0x38 /* Measurement 2 selects */
#define MAX86178_REG_MEAS2CFG1      0x39 /* Measurement 2 configuration 1 */
#define MAX86178_REG_MEAS2CFG2      0x3a /* Measurement 2 configuration 2 */
#define MAX86178_REG_MEAS2CFG3      0x3b /* Measurement 2 configuration 3 */
#define MAX86178_REG_MEAS2CFG4      0x3c /* Measurement 2 configuration 4 */
#define MAX86178_REG_MEAS2CFG5      0x3d /* Measurement 2 configuration 5 */
#define MAX86178_REG_MEAS2LEDA      0x3e /* Measurement 2 LEDA current */
#define MAX86178_REG_MEAS2LEDB      0x3f /* Measurement 2 LEDB current */
#define MAX86178_REG_MEAS3SEL       0x40 /* Measurement 3 selects */
#define MAX86178_REG_MEAS3CFG1      0x41 /* Measurement 3 configuration 1 */
#define MAX86178_REG_MEAS3CFG2      0x42 /* Measurement 3 configuration 2 */
#define MAX86178_REG_MEAS3CFG3      0x43 /* Measurement 3 configuration 3 */
#define MAX86178_REG_MEAS3CFG4      0x44 /* Measurement 3 configuration 4 */
#define MAX86178_REG_MEAS3CFG5      0x45 /* Measurement 3 configuration 5 */
#define MAX86178_REG_MEAS3LEDA      0x46 /* Measurement 3 LEDA current */
#define MAX86178_REG_MEAS3LEDB      0x47 /* Measurement 3 LEDB current */
#define MAX86178_REG_MEAS4SEL       0x48 /* Measurement 4 selects */
#define MAX86178_REG_MEAS4CFG1      0x49 /* Measurement 4 configuration 1 */
#define MAX86178_REG_MEAS4CFG2      0x4a /* Measurement 4 configuration 2 */
#define MAX86178_REG_MEAS4CFG3      0x4b /* Measurement 4 configuration 3 */
#define MAX86178_REG_MEAS4CFG4      0x4c /* Measurement 4 configuration 4 */
#define MAX86178_REG_MEAS4CFG5      0x4d /* Measurement 4 configuration 5 */
#define MAX86178_REG_MEAS4LEDA      0x4e /* Measurement 4 LEDA current */
#define MAX86178_REG_MEAS4LEDB      0x4f /* Measurement 4 LEDB current */
#define MAX86178_REG_MEAS5SEL       0x50 /* Measurement 5 selects */
#define MAX86178_REG_MEAS5CFG1      0x51 /* Measurement 5 configuration 1 */
#define MAX86178_REG_MEAS5CFG2      0x52 /* Measurement 5 configuration 2 */
#define MAX86178_REG_MEAS5CFG3      0x53 /* Measurement 5 configuration 3 */
#define MAX86178_REG_MEAS5CFG4      0x54 /* Measurement 5 configuration 4 */
#define MAX86178_REG_MEAS5CFG5      0x55 /* Measurement 5 configuration 5 */
#define MAX86178_REG_MEAS5LEDA      0x56 /* Measurement 5 LEDA current */
#define MAX86178_REG_MEAS5LEDB      0x57 /* Measurement 5 LEDB current */
#define MAX86178_REG_MEAS6SEL       0x58 /* Measurement 6 selects */
#define MAX86178_REG_MEAS6CFG1      0x59 /* Measurement 6 configuration 1 */
#define MAX86178_REG_MEAS6CFG2      0x5a /* Measurement 6 configuration 2 */
#define MAX86178_REG_MEAS6CFG3      0x5b /* Measurement 6 configuration 3 */
#define MAX86178_REG_MEAS6CFG4      0x5c /* Measurement 6 configuration 4 */
#define MAX86178_REG_MEAS6CFG5      0x5d /* Measurement 6 configuration 5 */
#define MAX86178_REG_MEAS6LEDA      0x5e /* Measurement 6 LEDA current */
#define MAX86178_REG_MEAS6LEDB      0x5f /* Measurement 6 LEDB current */
#define MAX86178_REG_THRESMEASSEL   0x70 /* Threshold measument selects */
#define MAX86178_REG_THRESHYST      0x71 /* Threshold hysteresis */
#define MAX86178_REG_PPGTHRES1HI    0x72 /* Upper threshold 1 */
#define MAX86178_REG_PPGTHRES1LO    0x73 /* Lower threshold 1 */
#define MAX86178_REG_PPGTHRES2HI    0x74 /* Upper threshold 2 */
#define MAX86178_REG_PPGTHRES2LO    0x75 /* Lower threshold 2 */
#define MAX86178_REG_ECGCFG1        0x80 /* ECG configuration 1 */
#define MAX86178_REG_ECGCFG2        0x81 /* ECG configuration 2 */
#define MAX86178_REG_ECGCFG3        0x82 /* ECG configuration 3 */
#define MAX86178_REG_ECGCFG4        0x83 /* ECG configuration 4 */
#define MAX86178_REG_ECGCALCFG1     0x84 /* ECG calibration configuration 1 */
#define MAX86178_REG_ECGCALCFG2     0x85 /* ECG calibration configuration 2 */
#define MAX86178_REG_ECGCALHIGHL    0x85 /* Alternative name of ECGCALCFG2 */
#define MAX86178_REG_ECGCALCFG3     0x86 /* ECG calibration configuration 3 */
#define MAX86178_REG_ECGLDCFG1      0x88 /* ECG lead detecion config 1 */
#define MAX86178_REG_ECGLDCFG2      0x89 /* ECG lead detecion config 2 */
#define MAX86178_REG_ECGLDBIAS1     0x90 /* ECG lead bias configuration 1 */
#define MAX86178_REG_RLDCFG1        0x92 /* Right leg drive config 1 */
#define MAX86178_REG_RLDCFG2        0x93 /* Right leg drive config 2 */
#define MAX86178_REG_BIOZCFG1       0xa0 /* BioZ configuration 1 */
#define MAX86178_REG_BIOZCFG2       0xa1 /* BioZ configuration 2 */
#define MAX86178_REG_BIOZCFG3       0xa2 /* BioZ configuration 3 */
#define MAX86178_REG_BIOZCFG4       0xa3 /* BioZ configuration 4 */
#define MAX86178_REG_BIOZCFG5       0xa4 /* BioZ configuration 5 */
#define MAX86178_REG_BIOZCFG6       0xa5 /* BioZ configuration 6 */
#define MAX86178_REG_BIOZCFG7       0xa6 /* BioZ configuration 7 */
#define MAX86178_REG_BIOZCFG8       0xa7 /* BioZ configuration 8 */
#define MAX86178_REG_BIOZTHRESL     0xa8 /* BioZ underrange threshold */
#define MAX86178_REG_BIOZTHRESH     0xa9 /* BioZ overrange threshold */
#define MAX86178_REG_BIOZMUXCFG1    0xaa /* BioZ multiplexer config 1 */
#define MAX86178_REG_BIOZMUXCFG2    0xab /* BioZ multiplexer config 2 */
#define MAX86178_REG_BIOZMUXCFG3    0xac /* BioZ multiplexer config 3 */
#define MAX86178_REG_BIOZMUXCFG4    0xad /* BioZ multiplexer config 4 */
#define MAX86178_REG_BIOZLDCFG1     0xb0 /* BioZ lead detection config 1 */
#define MAX86178_REG_BIOZLDOFFTHRES 0xb1 /* BioZ lead-off threshold */
#define MAX86178_REG_BIOZLDBIASCFG1 0xb4 /* BioZ lead bias configuration */
#define MAX86178_REG_RESPCFG1       0xb6 /* Respiration configuration */
#define MAX86178_REG_INT1EN1        0xc0 /* Interrupt1 enable 1 */
#define MAX86178_REG_INT1EN2        0xc1 /* Interrupt1 enable 2 */
#define MAX86178_REG_INT1EN3        0xc2 /* Interrupt1 enable 3 */
#define MAX86178_REG_INT1EN4        0xc3 /* Interrupt1 enable 4 */
#define MAX86178_REG_INT1EN5        0xc4 /* Interrupt1 enable 5 */
#define MAX86178_REG_INT2EN1        0xc5 /* Interrupt2 enable 1 */
#define MAX86178_REG_INT2EN2        0xc6 /* Interrupt2 enable 2 */
#define MAX86178_REG_INT2EN3        0xc7 /* Interrupt2 enable 3 */
#define MAX86178_REG_INT2EN4        0xc8 /* Interrupt2 enable 4 */
#define MAX86178_REG_INT2EN5        0xc9 /* Interrupt2 enable 5 */
#define MAX86178_REG_PARTID         0xff /* Part identifier */

/* Bits masks and functions of registers */

/* STAT1 bits functions */

/* 1: FIFO is almost full, i.e. has reached the threshold set by FIFOAFULL.
 * 0: Normal operation.
 */

#define MAX86178_STAT1_FIFOAFULL        (1 << 7)

/* 1: A complete PPG frame is ready in FIFO. 0: Normal operation. */

#define MAX86178_STAT1_PPGFRAMERDY      (1 << 6)

/* 1: New data is ready in FIFO. 0: Normal operation. */

#define MAX86178_STAT1_FIFODATARDY      (1 << 5)

/* 1: Ambient light current, i.e. dark current, is over or under range.
 * 0: Normal operation.
 */

#define MAX86178_STAT1_ALC_OVF          (1 << 4)

/* 1: Exposure measurement is over or under range. 0: Normal operation. */

#define MAX86178_STAT1_EXP_OVF          (1 << 3)

/* 1: PPG is above PPGTHRES2H or below PPGTHRES2L.
 * 0: PPG is within the range.
 */

#define MAX86178_STAT1_PPGTHRES2HILO    (1 << 2)

/* 1: PPG is above PPGTHRES1H or below PPGTHRES1L.
 * 0: PPG is within the range.
 */

#define MAX86178_STAT1_PPGTHRES1HILO    (1 << 1)

/* 1: VDVDD is too low, i.e. power is NOT ready. 0: Normal operation. */

#define MAX86178_STAT1_NPWRRDY          (1 << 0)

/* STAT2 bits functions */

/* 1: Invalid PPG cfg, i.e. frame rate is too fast. 0: Normal operation. */

#define MAX86178_STAT2_INVLDPPGCFG      (1 << 7)

/* 1: LED6_DRV pin voltage is below the compliance voltage needed to support
 *    the programmed current.
 * 0: LED6_DRV pin voltage can support the programmed current.
 */

#define MAX86178_STAT2_LED6COMPB        (1 << 5) /* LED6_DRV low voltage. */
#define MAX86178_STAT2_LED5COMPB        (1 << 4) /* See STAT2_LED6COMPB. */
#define MAX86178_STAT2_LED4COMPB        (1 << 3) /* See STAT2_LED6COMPB. */
#define MAX86178_STAT2_LED3COMPB        (1 << 2) /* See STAT2_LED6COMPB. */
#define MAX86178_STAT2_LED2COMPB        (1 << 1) /* See STAT2_LED6COMPB. */
#define MAX86178_STAT2_LED1COMPB        (1 << 0) /* See STAT2_LED6COMPB. */

/* STAT3 bits functions */

/* 1: PLL frequecy is unlocked. 0: PLL frequecy is locked. */

#define MAX86178_STAT3_FREQ_UNLOCK      (1 << 4)

/* 1: PLL frequecy is locked. 0: PLL frequecy is unlocked. */

#define MAX86178_STAT3_FREQ_LOCK        (1 << 3)

/* 1: PLL phase is unlocked. 0: PLL phase is locked. */

#define MAX86178_STAT3_PHASE_UNLOCK     (1 << 2)

/* 1: PLL phase is locked. 0: PLL phase is unlocked. */

#define MAX86178_STAT3_PHASE_LOCK       (1 << 1)

/* STAT4 bits functions */

/* 1: ECG lead-on condition has been detected. 0: Not detected. */

#define MAX86178_STAT4_ECG_LON          (1 << 7)

/* 1: ECG measurement is in fast recovery mode. 0: Normal operation. */

#define MAX86178_STAT4_ECG_FASTREC      (1 << 5)

/* 1: Right leg drive voltage has been out of range (<0.26V or >1.26V). */

#define MAX86178_STAT4_ECG_RLDOOR       (1 << 4)

/* 1: ECGP voltage is greater than upper threshold. 0: Normal operation. */

#define MAX86178_STAT4_ECGP_LOFFH       (1 << 3)

/* 1: ECGP voltage is less than lower threshold. 0: Normal operation. */

#define MAX86178_STAT4_ECGP_LOFFL       (1 << 2)

/* 1: ECGN voltage is greater than upper threshold. 0: Normal operation. */

#define MAX86178_STAT4_ECGN_LOFFH       (1 << 1)

/* 1: ECGN voltage is less than lower threshold. 0: Normal operation. */

#define MAX86178_STAT4_ECGN_LOFFL       (1 << 0)

/* STAT5 bits functions */

/* 1: BioZ lead-on condition has been detected. 0: Not detected. */

#define MAX86178_STAT5_BIOZ_LON         (1 << 7)

/* 1: Absolute BioZ reading has exceeded the high threshold. 0: Normal. */

#define MAX86178_STAT5_BIOZ_OVER        (1 << 6)

/* 1: Absolute BioZ reading has been below the low threshold. 0: Normal. */

#define MAX86178_STAT5_BIOZ_UNDR        (1 << 5)

/* 1: BioZ DRVN voltage peaks have been out of range. 0: Normal operation. */

#define MAX86178_STAT5_BIOZ_DRVOOR      (1 << 4)

/* 1: BIP voltage has been greater than the high threshold. 0: Normal. */

#define MAX86178_STAT5_BIP_LOFFH        (1 << 3)

/* 1: BIP voltage has been less than the low threshold. 0: Normal. */

#define MAX86178_STAT5_BIP_LOFFL        (1 << 2)

/* 1: BIN voltage has been greater than the high threshold. 0: Normal. */

#define MAX86178_STAT5_BIN_LOFFH        (1 << 1)

/* 1: BIN voltage has been less  than the low threshold. 0: Normal. */

#define MAX86178_STAT5_BIN_LOFFL        (1 << 0)

/* FIFOCNT1 bits masks */

#define MAX86178_FIFOCNT1_CNT_MSB_MASK  (1 << 7) /* MSB 1bit of FIFO count */
#define MAX86178_FIFOCNT1_OVF_CNT_MASK  0x7f     /* Overflow samples count */

/* FIFOCFG2 bits masks and functions */

/* Whether a marker tag is pushed to FIFO. */

#define MAX86178_FIFOCFG2_MARKER_MASK   (1 << 5) /* Mask */
#define MAX86178_FIFOCFG2_MARKER_DIS    (0 << 5) /* No marker is saved */
#define MAX86178_FIFOCFG2_MARKER_EN     (1 << 5) /* Marker is saved in FIFO */

/* Set 1 to trigger a FIFO flush, FIFOWRPT, FIFORDPT, FIFOCNT1 and FIFOCNT2
 * are reset to 0. Self clearing.
 */

#define MAX86178_FIFOCFG2_FLUSH         (1 << 4) /* Flush FIFO. */

/* Whether a FIFO reading will clear STAT1 bits FIFOAFULL， PPGFRAMERDY,
 * FIFODATARDY and their corresponding interrupts.
 */

#define MAX86178_FIFOCFG2_STAT_CLR_MASK (1 << 3) /* Mask */
#define MAX86178_FIFOCFG2_NSTATCLR      (0 << 3) /* Will NOT clear STAT1 */
#define MAX86178_FIFOCFG2_STATCLR       (1 << 3) /* Will Clear STAT1 */

/* FIFOAFULL in STAT1 is asserted when FIFO is almost full. It is cleared by
 * reading STAT1. When set 0, FIFOAFULL reasserts for every sample if FIFO
 * remains almost full; when set 1, it does not reassert until FIFO is read
 * and a new samples makes almost-full condition again.
 */

#define MAX86178_FIFOCFG2_FULLTYPE_MASK (1 << 2) /* Mask */
#define MAX86178_FIFOCFG2_AFULLALL      (0 << 2) /* Reassert on all samples */
#define MAX86178_FIFOCFG2_AFULLNEW      (1 << 2) /* Util a new almost-full. */

/* When FIFO is completely full, if set 0,FIFO stops filling, new samples are
 * lost, FIFOWRPT and FIFORDPT stop increasing; if set 1, FIFO automatically
 * rolls over, old samples are lost, FIFOWRPT and FIFORDPT go on increasing.
 */

#define MAX86178_FIFOCFG2_ROLL_MASK     (1 << 1) /* Mask */
#define MAX86178_FIFOCFG2_NROLL         (0 << 1) /* Stops if FIFO is full. */
#define MAX86178_FIFOCFG2_ROLL          (1 << 1) /* Rolls if FIFO is full. */

/* SYSSYNC bits masks and functions */

/* Set 1 to reset timing subsystem, i.e. reset BIOZ_NDIV, ECG_NDIV and
 * ECG_FDIV, and send a pulse to TRIG pin if TRIG_FCFG=2. Self clearing.
 */

#define MAX86178_SYSSYNC_RESET          (1 << 7) /* Reset timing subsystem. */

/* SYSCFG1 bits masks and functions */

/* Enable I2C or not. If enabled, CSB/I2C_SEL pin selects I2C/SPI. */

#define MAX86178_SYSCFG1_I2C_DIS_MASK   (1 << 6) /* Mask */
#define MAX86178_SYSCFG1_I2C_EN         (0 << 6) /* I2C enabled. */
#define MAX86178_SYSCFG1_I2C_DIS        (1 << 6) /* I2C disabled. SPI only. */

/* Whether ECG to PPG timing data is saved in FIFO. */

#define MAX86178_SYSCFG1_EPTIMING_MASK  (1 << 5) /* Mask */
#define MAX86178_SYSCFG1_EPTIMING_DIS   (0 << 5) /* Timing data NOT saved. */
#define MAX86178_SYSCFG1_EPTIMING_EN    (1 << 5) /* Timing data saved. */

/* Whether BioZ to PPG timing data is saved in FIFO. */

#define MAX86178_SYSCFG1_BPTIMING_MASK  (1 << 4) /* Mask */
#define MAX86178_SYSCFG1_BPTIMING_DIS   (0 << 4) /* Timing data NOT saved. */
#define MAX86178_SYSCFG1_BPTIMING_EN    (1 << 4) /* Timing data saved. */

/* Whether ECG to BioZ timing data is saved in FIFO. */

#define MAX86178_SYSCFG1_EBTIMING_MASK  (1 << 3) /* Mask */
#define MAX86178_SYSCFG1_EBTIMING_DIS   (0 << 3) /* Timing data NOT saved. */
#define MAX86178_SYSCFG1_EBTIMING_EN    (1 << 3) /* Timing data saved. */

/* Set chip in shutdown mode or normal mode, and get the mode */

#define MAX86178_SYSCFG1_SHDN_MASK      (1 << 1) /* Mask */
#define MAX86178_SYSCFG1_NORMAL         (0 << 1) /* Chip in normal mode. */
#define MAX86178_SYSCFG1_SHDN           (1 << 1) /* Chip in shutdown mode. */

/* Set 1 cause all register values to power-on-reset state. Self clearing. */

#define MAX86178_SYSCFG1_RESET          (1 << 0) /* Reset all registers. */

/* SYSCFG2 bits masks and functions */

/* Whether bypass the delays associated with bits in STAT4 adn STAT5. These
 * bits are threshold, lead-on, and lead-off features of the ECG and BioZ.
 */

#define MAX86178_SYSCFG2_BYP_DLY_MASK   (1 << 7) /* Mask */
#define MAX86178_SYSCFG2_EN_DLY         (0 << 7) /* Normal, delays enable */
#define MAX86178_SYSCFG2_BYP_DLY        (1 << 7) /* Delays bypassed */

/* Divide ratio for ECG sample rate, for outputing an async pulse on TRIG.
 * The ratio is the value + 1, e.g. 0x1f means divided by 32.
 */

#define MAX86178_SYSCFG2_ECGSYNCFQ_MASK 0x1f     /* Mask */

/* PINFUNC bits masks and functions */

/* TRIG pin function configuration, including direction and function.
 * 0: If PPGSYNCMODE = 1, TRIG input pin is used for external PPG sync. PPG
 *    measurement is in one-shot mode, where a frame begins upon an active
 *    edge on TRIG input.
 * 1: When PLL_EN = 1, TRIG is used to receive a external timing-system reset
 *    pulse to reset this device's PLL.
 * 2: When SYSSYNC_RESET is trigged, TRIG output a timing-system reset pulse.
 * 3: Output ECG sample sync pulse at the frequency set by ECGSYNCFQ.
 * 4: Output LED_TX pulse. LED_TX is asserted 500ns before any of the 6 LED
 *    driver pins gets asserted. It may be used to switch a boost converter.
 * If broadcast feature is used for timing synchronizing, 2 or 3 are ignored.
 */

#define MAX86178_PINFUNC_TRIG_FCFG_MASK (7 << 5) /* Mask */
#define MAX86178_PINFUNC_TRIG_PPGSYNC   (0 << 5) /* In, external PPG sync */
#define MAX86178_PINFUNC_TRIG_PLLSYNC_S (1 << 5) /* In, PLL sync slave */
#define MAX86178_PINFUNC_TRIG_PLLSYNC_M (2 << 5) /* Out, PLL sync master */
#define MAX86178_PINFUNC_TRIG_ECGSYNC   (3 << 5) /* Out, ECG sync pulse */
#define MAX86178_PINFUNC_TRIG_LED_TX    (4 << 5) /* Out, LED_TX pulse */

/* TRIG pin input active edge. */

#define MAX86178_PINFUNC_TRIG_ICFG_MASK (1 << 4) /* Mask */
#define MAX86178_PINFUNC_TRIG_FALLING   (0 << 4) /* Active edge is falling. */
#define MAX86178_PINFUNC_TRIG_RISING    (1 << 4) /* Active edge is rising. */

/* INT2 pin function configuration
 * 0: Disabled.
 * 1: INT2 is enabled and is cleared by reading STATx or FIFO.
 * 2: INT2 is enabled and is cleared automatically after 30.5us, or by
 *    reading STATx or FIFO.
 * 3: INT2 is enabled and is cleared automatically after 244us, or by
 *    reading STATx or FIFO.
 */

#define MAX86178_PINFUNC_INT2_FCFG_MASK (3 << 2) /* Mask */
#define MAX86178_PINFUNC_INT2_DISABLE   (0 << 2) /* Disabled. */
#define MAX86178_PINFUNC_INT2_RDCLR     (1 << 2) /* Enabled, reading-clear. */
#define MAX86178_PINFUNC_INT2_AUTOFAST  (2 << 2) /* Enabled, 30.5us-clear. */
#define MAX86178_PINFUNC_INT2_AUTOSLOW  (3 << 2) /* Enabled, 244us-clear. */

/* INT1 pin function configuration. See INT2_FCFG for details. */

#define MAX86178_PINFUNC_INT1_FCFG_MASK (3 << 0) /* Mask */
#define MAX86178_PINFUNC_INT1_DISABLE   (0 << 0) /* Disabled. */
#define MAX86178_PINFUNC_INT1_RDCLR     (1 << 0) /* Enabled, reading-clear. */
#define MAX86178_PINFUNC_INT1_AUTOFAST  (2 << 0) /* Enabled, 30.5us-clear. */
#define MAX86178_PINFUNC_INT1_AUTOSLOW  (3 << 0) /* Enabled, 244us-clear. */

/* PINCFG bits masks and functions */

/* TRIG pin output drive type configuration.
 * 0: Open-drain drive, and active level is low.
 * 1: Active drive to IOVDD or DGND, and active level is high.
 * 2: Active drive to IOVDD or DGND, and active level is low.
 */

#define MAX86178_PINCFG_TRIG_OCFG_MASK  (3 << 6) /* Mask */
#define MAX86178_PINCFG_TRIG_OD         (0 << 6) /* Open-drain, active-low */
#define MAX86178_PINCFG_TRIG_PP_HIGH    (1 << 6) /* Active DRV, active-high */
#define MAX86178_PINCFG_TRIG_PP_LOW     (2 << 6) /* Active DRV, active-low */

/* INT2 pin output drive type configuration. See TRIG_OCFG for details. */

#define MAX86178_PINCFG_INT2_OCFG_MASK  (3 << 2) /* Mask */
#define MAX86178_PINCFG_INT2_OD         (0 << 2) /* Open-drain, active-low. */
#define MAX86178_PINCFG_INT2_PP_HIGH    (1 << 2) /* Active DRV, active-high */
#define MAX86178_PINCFG_INT2_PP_LOW     (2 << 2) /* Active DRV, active-low */

/* INT1 pin output drive type configuration. See TRIG_OCFG for details. */

#define MAX86178_PINCFG_INT1_OCFG_MASK  (3 << 0) /* Mask */
#define MAX86178_PINCFG_INT1_OD         (0 << 0) /* Open-drain, active-low. */
#define MAX86178_PINCFG_INT1_PP_HIGH    (1 << 0) /* Active DRV, active-high */
#define MAX86178_PINCFG_INT1_PP_LOW     (2 << 0) /* Active DRV, active-low */

/* BCASTADDR bits masks and functions */

#define MAX86178_BCASTADDR_ADDR_MASK    0xfe     /* I2C broadcast addr mask */

/* Whether I2C broadcast is enabled, set or get. */

#define MAX86178_BCASTADDR_EN_MASK      (1 << 0) /* Mask */
#define MAX86178_BCASTADDR_DIS_BCAST    (0 << 0) /* Broadcast disabled. */
#define MAX86178_BCASTADDR_EN_BCAST     (1 << 0) /* Broadcast enabled. */

/* PLLCFG1 bits masks and functions */

#define MAX86178_PLLCFG1_MDIV_MSB_MASK  (3 << 6) /* MSB 2bits of PLL MDIV */

/* Select iime window for PLL phase lock detector  */

#define MAX86178_PLLCFG1_LOCK_WNDW_MASK (1 << 1) /* Mask */
#define MAX86178_PLLCFG1_LOCK_WNDW_1    (0 << 1) /* 1 PLL clock period */
#define MAX86178_PLLCFG1_LOCK_WNDW_2    (1 << 1) /* 2 PLL clock period */

/* Whether the PLL is enabled */

#define MAX86178_PLLCFG1_PLL_EN_MASK    (1 << 0) /* Mask */
#define MAX86178_PLLCFG1_PLL_DIS        (0 << 0) /* PLL is disabled. */
#define MAX86178_PLLCFG1_PLL_EN         (1 << 0) /* PLL is enabled. */

/* PLLCFG3 bits masks and functions */

/* BioZ NDIV choices */

#define MAX86178_PLLCFG3_BIOZNDIV_MASK  (3 << 6) /* Mask */
#define MAX86178_PLLCFG3_BIOZNDIV_256   (0 << 6) /* BioZ NDIV = 256 */
#define MAX86178_PLLCFG3_BIOZNDIV_512   (1 << 6) /* BioZ NDIV = 512 */
#define MAX86178_PLLCFG3_BIOZNDIV_1024  (2 << 6) /* BioZ NDIV = 1024 */

/* BioZ KDIV choices */

#define MAX86178_PLLCFG3_BIOZKDIV_MASK  0x0f     /* Mask */
#define MAX86178_PLLCFG3_BIOZKDIV_1     0        /* BioZ KDIV = 1 */
#define MAX86178_PLLCFG3_BIOZKDIV_2     1        /* BioZ KDIV = 2 */
#define MAX86178_PLLCFG3_BIOZKDIV_4     2        /* BioZ KDIV = 4 */
#define MAX86178_PLLCFG3_BIOZKDIV_8     3        /* BioZ KDIV = 8 */
#define MAX86178_PLLCFG3_BIOZKDIV_16    4        /* BioZ KDIV = 16 */
#define MAX86178_PLLCFG3_BIOZKDIV_32    5        /* BioZ KDIV = 32 */
#define MAX86178_PLLCFG3_BIOZKDIV_64    6        /* BioZ KDIV = 64 */
#define MAX86178_PLLCFG3_BIOZKDIV_128   7        /* BioZ KDIV = 128 */
#define MAX86178_PLLCFG3_BIOZKDIV_256   8        /* BioZ KDIV = 256 */
#define MAX86178_PLLCFG3_BIOZKDIV_512   9        /* BioZ KDIV = 512 */
#define MAX86178_PLLCFG3_BIOZKDIV_1024  10       /* BioZ KDIV = 1024 */
#define MAX86178_PLLCFG3_BIOZKDIV_2048  11       /* BioZ KDIV = 2048 */
#define MAX86178_PLLCFG3_BIOZKDIV_4096  12       /* BioZ KDIV = 4096 */
#define MAX86178_PLLCFG3_BIOZKDIV_8192  13       /* BioZ KDIV = 8192 (max.) */

/* PLLCFG4 bits masks, offsets and functions */

#define MAX86178_PLLCFG4_ECGNDIVH_MASK  (7 << 5) /* MSB 3bits of ECG NDIV */
#define MAX86178_PLLCFG4_ECGNDIVH_OFST  5        /* NDIV MABs offset  */

/* ECG FDIV choices */

#define MAX86178_PLLCFG4_ECGFDIV_MASK   (7 << 0) /* Mask */
#define MAX86178_PLLCFG4_ECGFDIV_DIS    (0 << 0) /* ECG FDIV is disabled */
#define MAX86178_PLLCFG4_ECGFDIV_1      (1 << 0) /* ECG FDIV = 1 */
#define MAX86178_PLLCFG4_ECGFDIV_2      (2 << 0) /* ECG FDIV = 2 */
#define MAX86178_PLLCFG4_ECGFDIV_4      (3 << 0) /* ECG FDIV = 4 */
#define MAX86178_PLLCFG4_ECGFDIV_8      (4 << 0) /* ECG FDIV = 8 */
#define MAX86178_PLLCFG4_ECGFDIV_16     (5 << 0) /* ECG FDIV = 16 (max.) */

/* PLLCFG6 bits masks and functions */

/* Select REF_CLK source for PLL */

#define MAX86178_PLLCFG6_REFCLKSEL_MASK (1 << 6) /* Mask */
#define MAX86178_PLLCFG6_REFCLK_INT     (0 << 6) /* Internal oscillator */
#define MAX86178_PLLCFG6_REFCLK_EXT     (1 << 6) /* External clock input */

/* REF_CLK frequcey for PLL */

#define MAX86178_PLLCFG6_CLKFRQSEL_MASK (1 << 5) /* Mask */
#define MAX86178_PLLCFG6_REFCLK_32K     (0 << 5) /* REF_CLK =  32.0 kHz */
#define MAX86178_PLLCFG6_REFCLK_32768   (1 << 5) /* REF_CLK =  32.768 kHz */

#define MAX86178_PLLCFG6_CLKMODIFY 0X1f     /* Clock modifying factor */

/* PPGCFG1 bits masks and functions */

#define MAX86178_PPGCFG1_MEAS6EN_MASK   (1 << 5) /* Mask of if MEAS6 enable */
#define MAX86178_PPGCFG1_MEAS6DIS       (0 << 5) /* MEAS6 disabled */
#define MAX86178_PPGCFG1_MEAS6EN        (1 << 5) /* MEAS6 enabled */

#define MAX86178_PPGCFG1_MEAS5EN_MASK   (1 << 4) /* Mask of if MEAS5 enable */
#define MAX86178_PPGCFG1_MEAS5DIS       (0 << 4) /* MEAS5 disabled */
#define MAX86178_PPGCFG1_MEAS5EN        (1 << 4) /* MEAS5 enabled */

#define MAX86178_PPGCFG1_MEAS4EN_MASK   (1 << 3) /* Mask of if MEAS4 enable */
#define MAX86178_PPGCFG1_MEAS4DIS       (0 << 3) /* MEAS4 disabled */
#define MAX86178_PPGCFG1_MEAS4EN        (1 << 3) /* MEAS4 enabled */

#define MAX86178_PPGCFG1_MEAS3EN_MASK   (1 << 2) /* Mask of if MEAS3 enable */
#define MAX86178_PPGCFG1_MEAS3DIS       (0 << 2) /* MEAS3 disabled */
#define MAX86178_PPGCFG1_MEAS3EN        (1 << 2) /* MEAS3 enabled */

#define MAX86178_PPGCFG1_MEAS2EN_MASK   (1 << 1) /* Mask of if MEAS2 enable */
#define MAX86178_PPGCFG1_MEAS2DIS       (0 << 1) /* MEAS2 disabled */
#define MAX86178_PPGCFG1_MEAS2EN        (1 << 1) /* MEAS2 enabled */

#define MAX86178_PPGCFG1_MEAS1EN_MASK   (1 << 0) /* Mask of if MEAS1 enable */
#define MAX86178_PPGCFG1_MEAS1DIS       (0 << 0) /* MEAS1 disabled */
#define MAX86178_PPGCFG1_MEAS1EN        (1 << 0) /* MEAS1 enabled */

/* PPGCFG2 bits masks and functions */

/* Select PPG frame synchronization mode.
 * 0: Free-running mode. Frame sync pulse is generated by internally by
 *    PPG_FR_CLK and FR_CLK_DIV.
 * 1: One-shot mode. A frame begins at an active edge on TRIG pin, when must
 *    set TRIG_FCFG = 0.
 */

#define MAX86178_PPGCFG2_SYNCMODE_MASK  (1 << 5) /* Mask */
#define MAX86178_PPGCFG2_SYNCMODE_INT   (0 << 5) /* Sync on internal clock */
#define MAX86178_PPGCFG2_SYNCMODE_EXT   (1 << 5) /* SYnc on external pulse */

/* Enable/disable PPG channel 2 */

#define MAX86178_PPGCFG2_PPG2PWRDN_MASK (1 << 3) /* Mask */
#define MAX86178_PPGCFG2_PPG2EN         (0 << 3) /* PPG channel2 enable */
#define MAX86178_PPGCFG2_PPG2PWRDN      (1 << 3) /* PPG channel2 power down */

/* Enable/disable PPG channel 2 */

#define MAX86178_PPGCFG2_PPG1PWRDN_MASK (1 << 2) /* Mask */
#define MAX86178_PPGCFG2_PPG1EN         (0 << 2) /* PPG channel1 enable */
#define MAX86178_PPGCFG2_PPG1PWRDN      (1 << 2) /* PPG channel1 power down */

/* PPGCFG3 bits masks and functions */

/* Enable/disable phtodiodes swapping when MWBA_EN = 1. This bit is ignored
 * when MWBA_EN = 0. When swapping is enabled, the photodiodes assigned to
 * PPG1 are swapped with the photodiodes assigned to PPG2 after burst average
 * completes.
 */
#define MAX86178_PPGCFG3_PD_SWAP_MASK   (1 << 7) /* Mask */
#define MAX86178_PPGCFG3_PD_SWAP        (0 << 7) /* PD is swapped */
#define MAX86178_PPGCFG3_PD_NSWAP       (1 << 7) /* PD is not swapped */

/* Set the number of adjacent samples from each individual PPG channel that
 * are averaged on-chip before being written to the FIFO. It's ignored when
 * MWBA_EN = 1. It's must be set 0 when threshold interrupts are enabled or
 * when external frame sync is used.
 */

#define MAX86178_PPGCFG3_SMP_AVE_MASK   (7 << 4) /* Mask */
#define MAX86178_PPGCFG3_SMP_AVE_DIS    (0 << 4) /* No averaging */
#define MAX86178_PPGCFG3_SMP_AVE_2      (1 << 4) /* Samples averaged = 2 */
#define MAX86178_PPGCFG3_SMP_AVE_4      (2 << 4) /* Samples averaged = 4 */
#define MAX86178_PPGCFG3_SMP_AVE_8      (3 << 4) /* Samples averaged = 8 */
#define MAX86178_PPGCFG3_SMP_AVE_16     (4 << 4) /* Samples averaged = 16 */

/* Enable/disable the front-end analog ambient-light cancelation circuit for
 * PPG measurements. Note that this bit does not alter the digital ambient-
 * light cancelation.
 */

#define MAX86178_PPGCFG3_ALC_DIS_MASK   (1 << 3) /* Mask */
#define MAX86178_PPGCFG3_ALC_DIS        (0 << 3) /* ALC enable */
#define MAX86178_PPGCFG3_ALC_EN         (1 << 3) /* ALC disable */

/* Enable/disable the multiple wavelength burst-average mode. In this mode,
 * the ADC conversion sequence is altered for achieving high SNR.
 */

#define MAX86178_PPGCFG3_MWBA_EN_MASK   (1 << 2) /* Mask */
#define MAX86178_PPGCFG3_MWBA_DIS       (0 << 2) /* MWBA disabled. */
#define MAX86178_PPGCFG3_MWBA_EN        (1 << 2) /* MWBA enabled. */

/* Whether pushes raw data, i.e. ambient conversion and exposure conversion,
 * to the FIFO separately. Setting 1 inhibits the digital ambient cancelation
 * and allows a customized ambient rejection algorithm. When set to 1,
 * PROXAUTO, THRES1_MEAS_SEL and THRES2_MEAS_SEL should be 0.
 */

#define MAX86178_PPGCFG3_RAWDATA_MASK   (1 << 1) /* Mask */
#define MAX86178_PPGCFG3_RAWDATA_DIS    (0 << 1) /* Raw data is not saved */
#define MAX86178_PPGCFG3_RAWDATA_EN     (1 << 1) /* Raw data is saved */

/* Whether all enabled PPG measurements use a unique configuration or use the
 * configuration settings defined in the MEAS1 registers (0x31 to 0x37).
 * Setting this bit to 1 allows for reduced setup writes.
 */

#define MAX86178_PPGCFG3_SAMECFG_MASK   (1 << 0) /* Mask */
#define MAX86178_PPGCFG3_UNIQCFG        (0 << 0) /* Each MEASx uses own cfg */
#define MAX86178_PPGCFG3_SAMECFG        (1 << 0) /* The same as MEAS1 cfgs*/

/* PPGCFG4 bits masks and functions */

/* Enable/disable MEAS6 data to be saved in the FIFO when PROXAUTO = 1. If
 * PROXAUTO is set to 0, this bit is ignored.
 */

#define MAX86178_PPGCFG4_PROXDATA_MASK  (1 << 5) /* Mask */
#define MAX86178_PPGCFG4_PROXDATA_DIS   (0 << 5) /* MEAS6 data is not saved */
#define MAX86178_PPGCFG4_PROXDATA_EN    (1 << 5) /* MEAS6 data is saved */

/* Enable/disable automatic proximity detect mode. */

#define MAX86178_PPGCFG4_PROXAUTO_MASK  (1 << 4) /* Mask */
#define MAX86178_PPGCFG4_PROXAUTO_DIS   (0 << 4) /* Normal mode */
#define MAX86178_PPGCFG4_PROXAUTO_EN    (1 << 4) /* Auto-PROX detect mode */

/* PDBIAS bits masks and functions */

/* Select the bias current for the photodiode. The current should be adjusted
 * according to the photodiode capacitance.
 * PDBIAS  |  Range of Photodiode Capacitance (pF)
 *  0x0    |   Not recommended
 *  0x1    |   0 to 125
 *  0x2    |   125 to 250
 *  0x3    |   250 to 500
 */

#define MAX86178_PDBIAS_PD4_MASK        (3 << 6) /* Mask: PD4 bias current */
#define MAX86178_PDBIAS_PD4_0_125PF     (1 << 6) /* 0pF < PD4 < 125pF */
#define MAX86178_PDBIAS_PD4_125_250PF   (2 << 6) /* 125pF < PD4 < 250pF */
#define MAX86178_PDBIAS_PD4_250_500PF   (3 << 6) /* 250pF < PD4 < 500pF */

#define MAX86178_PDBIAS_PD3_MASK        (3 << 4) /* Mask: PD3 bias current */
#define MAX86178_PDBIAS_PD3_0_125PF     (1 << 4) /* 0pF < PD3 < 125pF */
#define MAX86178_PDBIAS_PD3_125_250PF   (2 << 4) /* 125pF < PD3 < 250pF */
#define MAX86178_PDBIAS_PD3_250_500PF   (3 << 4) /* 250pF < PD3 < 500pF */

#define MAX86178_PDBIAS_PD2_MASK        (3 << 2) /* Mask: PD2 bias current */
#define MAX86178_PDBIAS_PD2_0_125PF     (1 << 2) /* 0pF < PD2 < 125pF */
#define MAX86178_PDBIAS_PD2_125_250PF   (2 << 2) /* 125pF < PD2 < 250pF */
#define MAX86178_PDBIAS_PD2_250_500PF   (3 << 2) /* 250pF < PD2 < 500pF */

#define MAX86178_PDBIAS_PD1_MASK        (3 << 0) /* Mask: PD1 bias current */
#define MAX86178_PDBIAS_PD1_0_125PF     (1 << 0) /* 0pF < PD1 < 125pF */
#define MAX86178_PDBIAS_PD1_125_250PF   (2 << 0) /* 125pF < PD1 < 250pF */
#define MAX86178_PDBIAS_PD1_250_500PF   (3 << 0) /* 250pF < PD1 < 500pF */

/* MEASxSEL bits masks and functions */

/* Enable/disable direct ambient measurement. When it's set to 1,
 * MEASXSEL_DRVA and MEASXSEL_DRVB are ignored.
 */

#define MAX86178_MEASXSEL_AMB_MASK      (1 << 6) /* Mask */
#define MAX86178_MEASXSEL_AMB_DIS       (0 << 6) /* Normal mode */
#define MAX86178_MEASXSEL_AMB_EN        (1 << 6) /* Direct ambient mode */

/* Select the LEDn_DRV pin (n = 1 to 6) driven by LED driver B */

#define MAX86178_MEASXSEL_DRVB_MASK     (7 << 3) /* Mask */
#define MAX86178_MEASXSEL_DRVB_LED1     (0 << 3) /* Driver B to LED1_DRV */
#define MAX86178_MEASXSEL_DRVB_LED2     (1 << 3) /* Driver B to LED2_DRV */
#define MAX86178_MEASXSEL_DRVB_LED3     (2 << 3) /* Driver B to LED3_DRV */
#define MAX86178_MEASXSEL_DRVB_LED4     (3 << 3) /* Driver B to LED4_DRV */
#define MAX86178_MEASXSEL_DRVB_LED5     (4 << 3) /* Driver B to LED5_DRV */
#define MAX86178_MEASXSEL_DRVB_LED6     (5 << 3) /* Driver B to LED6_DRV */

/* Select the LEDn_DRV pin (n = 1 to 6) driven by LED driver A */

#define MAX86178_MEASXSEL_DRVA_MASK     (7 << 0) /* Mask */
#define MAX86178_MEASXSEL_DRVA_LED1     (0 << 0) /* Driver A to LED1_DRV */
#define MAX86178_MEASXSEL_DRVA_LED2     (1 << 0) /* Driver A to LED2_DRV */
#define MAX86178_MEASXSEL_DRVA_LED3     (2 << 0) /* Driver A to LED3_DRV */
#define MAX86178_MEASXSEL_DRVA_LED4     (3 << 0) /* Driver A to LED4_DRV */
#define MAX86178_MEASXSEL_DRVA_LED5     (4 << 0) /* Driver A to LED5_DRV */
#define MAX86178_MEASXSEL_DRVA_LED6     (5 << 0) /* Driver A to LED6_DRV */

/* MEASxCFG1 bits masks and functions */

/* Enable/disable the SINC3 decimation filter for the PPG ADC. If it's set to
 * 1, MEASxCFG1_TINT must be set to 3 and MEASx_FILT2_SEL must be set to 0.
 */

#define MAX86178_MEASXCFG1_SINC3_MASK   (1 << 7) /* Mask */
#define MAX86178_MEASXCFG1_SINC3_DIS    (0 << 7) /* SINC3 filter isn't used */
#define MAX86178_MEASXCFG1_SINC3_EN     (1 << 7) /* SINC3 filter is used */

/* Select the decimation filter for PPG ADC.
 * 0: third-order decimation filter is used.
 * 1: second-order decimation filter is used, MEASxCFG1_SINC3 must be set to
 *    0 and MEASxCFG1_TINT must be 0x3.
 */

#define MAX86178_MEASXCFG1_FILT2_MASK   (1 << 6) /* Mask */
#define MAX86178_MEASXCFG1_FILT3        (1 << 6) /* Use 3rd-order filter */
#define MAX86178_MEASXCFG1_FILT2        (1 << 6) /* Use 2nd-order filter */

/* Select the digital ambient light rejection method to be used.
 * 0: Central difference method (CDM) is used.
 * 1: Forward difference method (FDM) is used.
 */

#define MAX86178_MEASXCFG1_FILT_MASK    (1 << 5) /* Mask */
#define MAX86178_MEASXCFG1_FILT_CDM     (0 << 5) /* CDM is used */
#define MAX86178_MEASXCFG1_FILT_FDM     (1 << 5) /* FDM is used */

/* Select the integration time of PPG ADCs. The choices are:
 * TINT | With 3rd-order filter | With 2nd-order filter
 * 0x0  |  14.6 us              |  Not applicable
 * 0x1  |  29.2 us              |  Not applicable
 * 0x2  |  58.6 us              |  Not applicable
 * 0x3  |  117 us               |  118.2 us
 */

#define MAX86178_MEASXCFG1_TINT_MASK    (3 << 3) /* Mask */
#define MAX86178_MEASXCFG1_TINT_14p6    (0 << 3) /* TINT = 14.6 us */
#define MAX86178_MEASXCFG1_TINT_29p2    (1 << 3) /* TINT = 29.2 us */
#define MAX86178_MEASXCFG1_TINT_58p6    (2 << 3) /* TINT = 58.6 us */
#define MAX86178_MEASXCFG1_TINT_117     (3 << 3) /* TINT = 117 or 118.2 us */

/* Select the number of exposures to be averaged in order to get one
 * measurement sample. When setting it to any value other than 0,
 * MEASxCFG1_FILT must be 0. With MEASxCFG1_AVER, each sample in FIFO is a
 * computed average of (2 x 2^MEASxCFG1_AVER + 1) ADC conversions of
 * interleaved ambient (2^MEASx_AVER + 1) and exposure (2^MEASx_AVER)
 * measurements. When MEASxCFG1_FILT = 1, these bits are ignored.
 * An exception: When PPGCFG3_MWBA_EN and MEASXCFG1_AVER = 7, number of LED
 * pulses is not 2^7 = 128 but 2^6 = 64.
 */

#define MAX86178_MEASXCFG1_AVER_MASK    (7 << 0) /* Mask */
#define MAX86178_MEASXCFG1_AVER_DIS     (0 << 0) /* No averaging */
#define MAX86178_MEASXCFG1_AVER_2       (1 << 0) /* 2 exposure samples */
#define MAX86178_MEASXCFG1_AVER_4       (2 << 0) /* 4 exposure samples */
#define MAX86178_MEASXCFG1_AVER_8       (3 << 0) /* 8 exposure samples */
#define MAX86178_MEASXCFG1_AVER_16      (4 << 0) /* 16 exposure samples */
#define MAX86178_MEASXCFG1_AVER_32      (5 << 0) /* 32 exposure samples */
#define MAX86178_MEASXCFG1_AVER_64      (6 << 0) /* 64 exposure samples */
#define MAX86178_MEASXCFG1_AVER_128     (7 << 0) /* 128 exposure samples */

/* MEASxCFG2 bits masks and functions */

/* Select the positive full-scale range of the PPG ADC on channel 1/2 */

#define MAX86178_MEASXCFG2_PPG2RGE_MASK (3 << 4) /* Mask */
#define MAX86178_MEASXCFG2_PPG2RGE_OFST 4        /* Bit offset */
#define MAX86178_MEASXCFG2_PPG2RGE_4UA  (0 << 4) /* Full-scale = 4.0 uA */
#define MAX86178_MEASXCFG2_PPG2RGE_8UA  (1 << 4) /* Full-scale = 8.0 uA */
#define MAX86178_MEASXCFG2_PPG2RGE_16UA (2 << 4) /* Full-scale = 16.0 uA */
#define MAX86178_MEASXCFG2_PPG2RGE_32UA (3 << 4) /* Full-scale = 32.0 uA */

#define MAX86178_MEASXCFG2_PPG1RGE_MASK (3 << 0) /* Mask */
#define MAX86178_MEASXCFG2_PPG1RGE_4UA  (0 << 0) /* Full-scale = 4.0 uA */
#define MAX86178_MEASXCFG2_PPG1RGE_8UA  (1 << 0) /* Full-scale = 8.0 uA */
#define MAX86178_MEASXCFG2_PPG1RGE_16UA (2 << 0) /* Full-scale = 16.0 uA */
#define MAX86178_MEASXCFG2_PPG1RGE_32UA (3 << 0) /* Full-scale = 32.0 uA */

/* MEASxCFG3 bits masks and functions */

/* Select the offset DAC current added to the ADC on PPG channel during the
 * exposure interval. This allows for a larger convertible exposure range for
 * ADC by sourcing some of the photodiode DC exposure current from the DAC.
 * The injected offset current to ADC (μA) = the value * 2 (μA).
 */

#define MAX86178_MEASXCFG3_PPG2OFF_MASK (15 << 4) /* Mask */
#define MAX86178_MEASXCFG3_PPG1OFF_MASK (15 << 0) /* Mask */

/* MEASxCFG4 bits masks and functions */

/* Select the time between dark and exposure samples for measurement. This
 * accomodates photodiodes with longer settling time. PD settling time should
 * always be more than LED settling. Note that for the same setting of both
 * MEASXCFG4_PDSETT and MEASXCFG4_LEDSETT, the photodiode settling time is
 * 0.1μs higher than the LED settling time, and thus, satisfies the
 * mentioned requirement of the higher PD settling time.
 */

#define MAX86178_MEASXCFG4_PDSETT_MASK  (3 << 6) /* Mask */
#define MAX86178_MEASXCFG4_PDSETT_7p8   (3 << 6) /* Settling time = 7.8 us */
#define MAX86178_MEASXCFG4_PDSETT_11p8  (3 << 6) /* Settling time = 11.8 us */
#define MAX86178_MEASXCFG4_PDSETT_15p8  (3 << 6) /* Settling time = 15.8 us */
#define MAX86178_MEASXCFG4_PDSETT_23p8  (3 << 6) /* Settling time = 23.8 us */

/* Select the delay from the rising edge of LED to the start of the exposure
 * ADC integration. This allows for the LED current to settle before the
 * start of ADC integration. LED settling time for a measurement must always
 * be less than the photodiode settling time for the same measurement. When
 * MWBA_EN is set to 1, this bit is ignored, and the LED settling time is
 * 0.1μs less than the photodiode settling time.
 */

#define MAX86178_MEASXCFG4_LEDSETT_MASK (3 << 2) /* Mask */
#define MAX86178_MEASXCFG4_LEDSETT_7p7  (3 << 2) /* Settling time = 7.7 us */
#define MAX86178_MEASXCFG4_LEDSETT_11p7 (3 << 2) /* Settling time = 11.7 us */
#define MAX86178_MEASXCFG4_LEDSETT_15p7 (3 << 2) /* Settling time = 15.7 us */
#define MAX86178_MEASXCFG4_LEDSETT_23p7 (3 << 2) /* Settling time = 23.7 us */

/* Select the drive current range for both LED current drivers. */

#define MAX86178_MEASXCFG4_LEDRGE_MASK  (3 << 0) /* Mask */
#define MAX86178_MEASXCFG4_LEDRGE_32MA  (0 << 0) /* LED FS range = 32 mA */
#define MAX86178_MEASXCFG4_LEDRGE_64MA  (1 << 0) /* LED FS range = 64 mA */
#define MAX86178_MEASXCFG4_LEDRGE_96MA  (2 << 0) /* LED FS range = 96 mA */
#define MAX86178_MEASXCFG4_LEDRGE_128MA (3 << 0) /* LED FS range = 128 mA */

/* MEASxCFG5 bits masks and functions */

/* Select to which optical channel the PDx input connects. */

#define MAX86178_MEASXCFG5_PD4SEL_MASK  (3 << 6) /* Mask */
#define MAX86178_MEASXCFG5_PD4SEL_NC    (0 << 6) /* PD4 is not selected. */
#define MAX86178_MEASXCFG5_PD4SEL_PPG1  (2 << 6) /* PD4 connects to PPG1. */
#define MAX86178_MEASXCFG5_PD4SEL_PPG2  (3 << 6) /* PD4 connects to PPG2. */

#define MAX86178_MEASXCFG5_PD3SEL_MASK  (3 << 4) /* Mask */
#define MAX86178_MEASXCFG5_PD3SEL_NC    (0 << 4) /* PD3 is not selected. */
#define MAX86178_MEASXCFG5_PD3SEL_PPG1  (2 << 4) /* PD3 connects to PPG1. */
#define MAX86178_MEASXCFG5_PD3SEL_PPG2  (3 << 4) /* PD3 connects to PPG2. */

#define MAX86178_MEASXCFG5_PD2SEL_MASK  (3 << 2) /* Mask */
#define MAX86178_MEASXCFG5_PD2SEL_NC    (0 << 2) /* PD2 is not selected. */
#define MAX86178_MEASXCFG5_PD2SEL_PPG1  (2 << 2) /* PD2 connects to PPG1. */
#define MAX86178_MEASXCFG5_PD2SEL_PPG2  (3 << 2) /* PD2 connects to PPG2. */

#define MAX86178_MEASXCFG5_PD1SEL_MASK  (3 << 0) /* Mask */
#define MAX86178_MEASXCFG5_PD1SEL_NC    (0 << 0) /* PD1 is not selected. */
#define MAX86178_MEASXCFG5_PD1SEL_PPG1  (2 << 0) /* PD1 connects to PPG1. */
#define MAX86178_MEASXCFG5_PD1SEL_PPG2  (3 << 0) /* PD1 connects to PPG2. */

/* THRESMEASSEL bits masks and functions */

/* Enable the threshold detect function and selects the PPG measurement for
 * the THRESHOLD2 function. If the threshold detect function is enabled,
 * PPGCFG3_RAWDATA and PPGCFG3_SMP_AVE must be set to 0.
 */

#define MAX86178_THRESMEASSEL_2_MASK    (7 << 4) /* Mask */
#define MAX86178_THRESMEASSEL_2_DIS     (0 << 4) /* Threshold2 disabled */
#define MAX86178_THRESMEASSEL_2_MEAS1   (1 << 4) /* MEAS1 for threshold2 */
#define MAX86178_THRESMEASSEL_2_MEAS2   (2 << 4) /* MEAS2 for threshold2 */
#define MAX86178_THRESMEASSEL_2_MEAS3   (3 << 4) /* MEAS3 for threshold2 */
#define MAX86178_THRESMEASSEL_2_MEAS4   (4 << 4) /* MEAS4 for threshold2 */
#define MAX86178_THRESMEASSEL_2_MEAS5   (5 << 4) /* MEAS5 for threshold2 */
#define MAX86178_THRESMEASSEL_2_MEAS6   (6 << 4) /* MEAS6 for threshold2 */

#define MAX86178_THRESMEASSEL_1_MASK    (7 << 0) /* Mask */
#define MAX86178_THRESMEASSEL_1_DIS     (0 << 0) /* Threshold2 disabled */
#define MAX86178_THRESMEASSEL_1_MEAS1   (1 << 0) /* MEAS1 for threshold2 */
#define MAX86178_THRESMEASSEL_1_MEAS2   (2 << 0) /* MEAS2 for threshold2 */
#define MAX86178_THRESMEASSEL_1_MEAS3   (3 << 0) /* MEAS3 for threshold2 */
#define MAX86178_THRESMEASSEL_1_MEAS4   (4 << 0) /* MEAS4 for threshold2 */
#define MAX86178_THRESMEASSEL_1_MEAS5   (5 << 0) /* MEAS5 for threshold2 */
#define MAX86178_THRESMEASSEL_1_MEAS6   (6 << 0) /* MEAS6 for threshold2 */

/* THRESHYST bits masks and functions */

/* Thess 2 bits select the optical channel for THRESHOLD 1 and 2 */

#define MAX86178_THRESHYST_2_SEL_MASK   (1 << 7) /* Mask */
#define MAX86178_THRESHYST_2_SEL_PPG1   (0 << 7) /* PPG1 for threshold2 */
#define MAX86178_THRESHYST_2_SEL_PPG2   (1 << 7) /* PPG2 for threshold2 */

#define MAX86178_THRESHYST_1_SEL_MASK   (1 << 6) /* Mask */
#define MAX86178_THRESHYST_1_SEL_PPG1   (0 << 6) /* PPG1 for threshold1 */
#define MAX86178_THRESHYST_1_SEL_PPG2   (1 << 6) /* PPG2 for threshold1 */

/* Selects the number of consecutive samples outside the limits defined by
 * THRESxHI and THRESxLO in order to trigger the threshold interrupt
 * THRESxHILO. TIMHYST applies to both instances of threshold interrupts.
 */

#define MAX86178_THRESHYST_TIMHYST_MASK (3 << 3) /* Mask: time hysteresis */
#define MAX86178_THRESHYST_TIMHYST_DIS  (0 << 3) /* Time hysteresis disable */
#define MAX86178_THRESHYST_TIMHYST_2    (1 << 3) /* 2 samples hysteresis */
#define MAX86178_THRESHYST_TIMHYST_4    (2 << 3) /* 4 samples hysteresis */
#define MAX86178_THRESHYST_TIMHYST_8    (3 << 3) /* 8 samples hysteresis */

/* Set the variation in ADC counts permitted when the THRESxHILO interrupt is
 * triggered. This value is in ADC counts and is applied at ±0.5 x LVLHYST
 * around the THRESHOLDxHI and THRESHOLDxLO. It applies to both instances of
 * threshold interrupts.
 */

#define MAX86178_THRESHYST_LVLHYST_MASK (7 << 7) /* Mask: Level hysteresis */
#define MAX86178_THRESHYST_LVLHYST_DIS  (0 << 0) /* Disable */
#define MAX86178_THRESHYST_LVLHYST_2    (1 << 0) /* Hysteresis lvl = 2LSB */
#define MAX86178_THRESHYST_LVLHYST_4    (2 << 0) /* Hysteresis lvl = 4LSB */
#define MAX86178_THRESHYST_LVLHYST_8    (3 << 0) /* Hysteresis lvl = 8LSB */
#define MAX86178_THRESHYST_LVLHYST_16   (4 << 0) /* Hysteresis lvl = 16LSB */
#define MAX86178_THRESHYST_LVLHYST_32   (5 << 0) /* Hysteresis lvl = 32LSB */
#define MAX86178_THRESHYST_LVLHYST_64   (6 << 0) /* Hysteresis lvl = 64LSB */
#define MAX86178_THRESHYST_LVLHYST_128  (7 << 0) /* Hysteresis lvl = 128LSB */

/* ECGCFG1 bits masks and functions */

/* These DEC_RATE bits set the decimation ratio for the ECG_ADC. */

#define MAX86178_ECGCFG1_DEC_RATE_MASK  (7 << 1) /* Mask*/
#define MAX86178_ECGCFG1_DEC_RATE_16    (0 << 1) /* Decimation ratio = 16 */
#define MAX86178_ECGCFG1_DEC_RATE_32    (1 << 1) /* Decimation ratio = 32 */
#define MAX86178_ECGCFG1_DEC_RATE_64    (2 << 1) /* Decimation ratio = 64 */
#define MAX86178_ECGCFG1_DEC_RATE_128   (3 << 1) /* Decimation ratio = 128 */
#define MAX86178_ECGCFG1_DEC_RATE_256   (4 << 1) /* Decimation ratio = 256 */
#define MAX86178_ECGCFG1_DEC_RATE_512   (5 << 1) /* Decimation ratio = 512 */

#define MAX86178_ECGCFG1_ECG_EN_MASK    (1 << 0) /* Enable/disable ECG */
#define MAX86178_ECGCFG1_ECG_DIS        (0 << 0) /* ECG is disabled */
#define MAX86178_ECGCFG1_ECG_EN         (1 << 0) /* ECG is enabled */

/* ECGCFG2 bits masks and functions */

#define MAX86178_ECGCFG2_IPOL_MASK      (1 << 7) /* Mask: input polarity */
#define MAX86178_ECGCFG2_IPOL_NINV      (0 << 7) /* Non-inverted polarity */
#define MAX86178_ECGCFG2_IPOL_INV       (1 << 7) /* Inverted polarity */

#define MAX86178_ECGCFG2_PGAGAIN_MASK   (7 << 4) /* Mask: gain for the PGA */
#define MAX86178_ECGCFG2_PGAGAIN_1      (0 << 4) /* PGA gain = 1 V/V */
#define MAX86178_ECGCFG2_PGAGAIN_2      (1 << 4) /* PGA gain = 2 v/v */
#define MAX86178_ECGCFG2_PGAGAIN_4      (2 << 4) /* PGA gain = 4 V/V */
#define MAX86178_ECGCFG2_PGAGAIN_8      (3 << 4) /* PGA gain = 8 V/V */
#define MAX86178_ECGCFG2_PGAGAIN_16     (7 << 4) /* PGA gain = 16 V/V */

/* INARGE selects the gain range of the ECG INA (input amplifier). INAGAIN
 * selects the gain of the ECG INA. The fina INA gains are:
 * INAGAIN  |  INARGE = 0  |  INARGE = 1  |  INARGE = 2  |  INARGE = 3
 *    0     |    10 V/V    |    7.5 V/V   |    5 V/V     |    2.5 V/V
 *    1     |    20 V/V    |    15 V/V    |    10 V/V    |    5 V/V
 *    2     |    40 V/V    |    30 V/V    |    20 V/V    |    10 V/V
 *    3     |    60 V/V    |    45 V/V    |    30 V/V    |    15 V/V
 */

#define MAX86178_ECGCFG2_INARGE_MASK    (3 << 2) /* Mask: range of the INA */
#define MAX86178_ECGCFG2_INARGE_0       (0 << 2) /* See comments above */
#define MAX86178_ECGCFG2_INARGE_1       (1 << 2) /* See comments above */
#define MAX86178_ECGCFG2_INARGE_2       (2 << 2) /* See comments above */
#define MAX86178_ECGCFG2_INARGE_3       (3 << 2) /* See comments above */
#define MAX86178_ECGCFG2_INAGAIN_MASK   (3 << 0) /* Mask: gain for the INA */
#define MAX86178_ECGCFG2_INAGAIN_0      (0 << 0) /* See comments above */
#define MAX86178_ECGCFG2_INAGAIN_1      (1 << 0) /* See comments above */
#define MAX86178_ECGCFG2_INAGAIN_2      (2 << 0) /* See comments above */
#define MAX86178_ECGCFG2_INAGAIN_3      (3 << 0) /* See comments above */

/* ECGCFG3 bits masks and functions */

/* IMP_HI selects the combined output impedance of CAPP and CAPN. This
 * impedance together with the value of the external capacitor connected
 * between CAPP and CAPN sets the HPF corner frequency of the ECG channel.
 * If it's set to 0, the input impedance is independent of INARGE setting;
 * if it's set to 1, the input impedance is dependent on INARGE setting. THe
 * The input impedances are:
 *   INARGE  |  IMP_HI = 0  |  IMP_HI = 1
 *    0x0    |   400 kOhm   |   400 kOhm
 *    0x1    |   400 kOhm   |   533 kOhm
 *    0x2    |   400 kOhm   |   800 kOhm
 *    0x3    |   400 kOhm   |   1600 kOhm
 */

#define MAX86178_ECGCFG3_IMP_HI_MASK    (1 << 3) /* Mask */
#define MAX86178_ECGCFG3_IMP_HI_FIXED   (0 << 3) /* Impendance is fixed. */
#define MAX86178_ECGCFG3_IMP_HI_VARY    (1 << 3) /* Impendance varys. */

/* AUTOREC enables analog automatic recovery mode in the ECG INA. When
 * enabled, the INA automatically enables the fast recovery buffers when the
 * INA is saturated.
 */

#define MAX86178_ECGCFG3_AUTOREC_MASK   (1 << 2) /* Mask */
#define MAX86178_ECGCFG3_AUTOREC_DIS    (0 << 2) /* Auto-recovery disabled */
#define MAX86178_ECGCFG3_AUTOREC_EN     (1 << 2) /* Auto-recovery enabled */

/* MUXSEL[1:0] defines how ECGP/ECGN inputs and RLD output are routed to the
 * ECG_EL1/2/3 pins. The selection table is shown below. The pins are
 * disconnected from the ECG/RLD circuit blocks by default. ECG_OPEN_P and
 * ECG_OPEN_N must also be set to 0 to connect the ECG inputs. ECGP and ECGN
 * connections can be swapped by the ECG_IPOL bit. For example, ECG_EL1 is
 * routed to the ECGP, and ECG_EL2 is routed to the ECGN when  MUXSEL = 0x1
 * if ECG_IPOL = 0. ECG_EL1 is routed to ECGN if ECG_IPOL = 1.
 * ECGCFG3_MUXSEL  |    ECGP    |    ECGN    |    RLD
 *      0x0        |     NC     |     NC     |     NC
 *      0x1        |   ECG_EL1  |   ECG_EL2  |   ECG_EL3
 *      0x2        |   ECG_EL3  |   ECG_EL1  |   ECG_EL2
 *      0x3        |   ECG_EL2  |   ECG_EL3  |   ECG_EL1
 */

#define MAX86178_ECGCFG3_MUXSEL_MASK    (3 << 2) /* Mask */
#define MAX86178_ECGCFG3_MUXSEL_NC      (0 << 0) /* All ECG input pins NC */
#define MAX86178_ECGCFG3_MUXSEL_EL123   (1 << 0) /* To EL1/2/3 respectively */
#define MAX86178_ECGCFG3_MUXSEL_EL312   (2 << 0) /* To EL3/1/2 respectively */
#define MAX86178_ECGCFG3_MUXSEL_EL231   (3 << 0) /* To EL2/3/1 respectively */

/* ECGCFG4 bits masks and functions */

/* These bits enables digital automatic fast recovery mode or manual mode in
 * the ECG INA. Manual fast recovery mode remains active once it is enabled
 * until it is manually disabled by setting FASTREC to 0x0. Automatic fast
 * recovery mode is activated when the ECG ADC count is outside of the
 * threshold set by REC_THRES for approximately 125ms, and remains active for
 * approximately 500ms.
 */

#define MAX86178_ECGCFG4_FASTREC_MASK   (3 << 6) /* Mask */
#define MAX86178_ECGCFG4_FASTREC_DIS    (0 << 6) /* No digital fast-rec */
#define MAX86178_ECGCFG4_FASTREC_MANUAL (1 << 6) /* Manual mode fast-rec */
#define MAX86178_ECGCFG4_FASTREC_AUTO   (2 << 6) /* Auto mode fast-rec */

/* If FASTREC is set to 0x2 and the output of an ECG measurement exceeds the
 * symmetric thresholds defined by ±(2048 x REC_THRES) for more than 125ms,
 * the fast recovery mode is automatically engaged. The default value 0x3f
 * corresponds to an upper threshold of 0x1F800 and a lower threshold of
 * 0x20800, or ±98.4% of full-scale.
 */

#define MAX86178_ECGCFG4_REC_THRES_MASK 0x3f     /* Mask */

/* ECGCALCFG1 bits masks and functions */

/* These bits adn ECGCALLSB consist ECG_CAL_HIGH[10:0], which determines the
 * time high (or duty cycle) for the calibration source when ECGCALCFG3_DUTY
 * is set to 0. Time high is calculated by the equation:
 * tHIGH = ECG_CAL_HIGH[10:0] x 1 / ECG_ADC_CLK,
 * where the tHIGH should be lower than the calibration frequency period set
 * by ECGCALCFG3_FREQ.
 */

#define MAX86178_ECGCALCFG1_HIGHH_MASK  (7 << 5) /* Mask: ECG_CAL_HIGH MSBs */

/* Select the frequency of the calibration source (fCAL), relative to the ECG
 * ADC clock. The fCALs are:
 * ECGCALCFG1_FREQ  |         fCAL
 *      0x0         |  ECG_ADC_CLK / 128
 *      0x1         |  ECG_ADC_CLK / 512
 *      0x2         |  ECG_ADC_CLK / 2048
 *      0x3         |  ECG_ADC_CLK / 8192
 *      0x4         |  ECG_ADC_CLK / 2^15
 *      0x5         |  ECG_ADC_CLK / 2^17
 *      0x6         |  ECG_ADC_CLK / 2^19
 *      0x7         |  ECG_ADC_CLK / 2^21
 */

#define MAX86178_ECGCALCFG1_FREQ_MASK   (7 << 2) /* Mask */
#define MAX86178_ECGCALCFG1_FREQ_128    (0 << 2) /* See table above. */
#define MAX86178_ECGCALCFG1_FREQ_512    (1 << 2) /* See table above. */
#define MAX86178_ECGCALCFG1_FREQ_2048   (2 << 2) /* See table above. */
#define MAX86178_ECGCALCFG1_FREQ_8192   (3 << 2) /* See table above. */
#define MAX86178_ECGCALCFG1_FREQ_2E15   (4 << 2) /* See table above. */
#define MAX86178_ECGCALCFG1_FREQ_2E17   (5 << 2) /* See table above. */
#define MAX86178_ECGCALCFG1_FREQ_2E19   (6 << 2) /* See table above. */
#define MAX86178_ECGCALCFG1_FREQ_2E21   (7 << 2) /* See table above. */

/* Selects between time-high and 50% duty modes of the calibration source */

#define MAX86178_ECGCALCFG1_DUTY_MASK   (1 << 1) /* Mask */
#define MAX86178_ECGCALCFG1_DUTY_VARY   (0 << 1) /* CAL_HIGH defines tHIGH */
#define MAX86178_ECGCALCFG1_DUTY_HALF   (1 << 1) /* Duty cycle is 50% */

/* When ECG_EN is set to 1, ECG_CAL_EN enables ECG calibration sources VCALP
 * and VCALN. Before enabling this, ensure that the input switches are
 * disconnected (OPEN_P and OPEN_N are set to 1 in register 0x86).
 */

#define MAX86178_ECGCALCFG1_EN_MASK     (1 << 0) /* Mask */
#define MAX86178_ECGCALCFG1_DIS         (0 << 0) /* Calibration disabled */
#define MAX86178_ECGCALCFG1_EN          (1 << 0) /* Calibration enabled */

/* ECGCALCFG3 bits masks and functions */

/* OPENP/OPENN controls the ECGP/ECGN input switch. These switched must be
 * connected (set to 0) to measure ECG signal, and must be disconnected (set
 * to 1) before enabling the calibration voltage sources.
 */

#define MAX86178_ECGCALCFG3_OPENP_MASK  (1 << 7) /* Mask: OPEN_P */
#define MAX86178_ECGCALCFG3_CONNECT_P   (0 << 7) /* ECGP connected to ECG */
#define MAX86178_ECGCALCFG3_OPEN_P      (1 << 7) /* ECGP isolated from ECG */

#define MAX86178_ECGCALCFG3_OPENN_MASK  (1 << 6) /* Mask: OPEN_N */
#define MAX86178_ECGCALCFG3_CONNECT_N   (0 << 6) /* ECGN connected to ECG */
#define MAX86178_ECGCALCFG3_OPEN_N      (1 << 6) /* ECGN isolated from ECG */

/* Select the mode of calibration source.
 * 0: Unipolar, sources swing between (VMID_ECG ± VCAL_MAG) and (VMID_ECG).
 * 1: Bipolar, sources swing between (VMID_ECG + VCAL_MAG) and
 *    (VMID_ECG - VCAL_MAG).
 */

#define MAX86178_ECGCALCFG3_MODE_MASK   (1 << 5) /* Mask: Calibration mode */
#define MAX86178_ECGCALCFG3_MODE_UNI    (0 << 5) /* Unipolar mode */
#define MAX86178_ECGCALCFG3_MODE_BI     (1 << 5) /* Bipolar mode */

/* Select the magnitude of the calibration sources. */

#define MAX86178_ECGCALCFG3_MAG_MASK    (1 << 4) /* Mask: CAL magnitude */
#define MAX86178_ECGCALCFG3_MAG_0p5     (0 << 4) /* VCAL_MAG = 0.5mV */
#define MAX86178_ECGCALCFG3_MAG_1       (1 << 4) /* VCAL_MAG = 1.0mV */

/* PSEL or NSEL selects which calibration voltage source is connected to the
 * ECGP/ECGN input.
 * 0: No calibration signal applied.
 * 1: ECGP/ECGN is connected to VMID_ECG
 * 2: ECGP/ECGN is connected to VCALP (available if ECGCALCFG1_EN)
 * 3: ECGP/ECGN is connected to VCALN (available if ECGCALCFG1_EN)
 */

#define MAX86178_ECGCALCFG3_PSEL_MASK   (3 << 2) /* Mask: ECG_CAL_P_SEL */
#define MAX86178_ECGCALCFG3_PSEL_DIS    (0 << 2) /* No calibration signal */
#define MAX86178_ECGCALCFG3_PSEL_VMID   (1 << 2) /* ECGP to VMID_ECG */
#define MAX86178_ECGCALCFG3_PSEL_VCALP  (2 << 2) /* ECGP connected to VCALP */
#define MAX86178_ECGCALCFG3_PSEL_VCALN  (3 << 2) /* ECGP connected to VCALN */

#define MAX86178_ECGCALCFG3_NSEL_MASK   (3 << 2) /* Mask: ECG_CAL_N_SEL */
#define MAX86178_ECGCALCFG3_NSEL_DIS    (0 << 2) /* No calibration signal */
#define MAX86178_ECGCALCFG3_NSEL_VMID   (1 << 2) /* ECGN to VMID_ECG */
#define MAX86178_ECGCALCFG3_NSEL_VCALP  (2 << 2) /* ECGN connected to VCALP */
#define MAX86178_ECGCALCFG3_NSEL_VCALN  (3 << 2) /* ECGN connected to VCALN */

/* ECGLDCFG1 bits masks and functions */

/* This bit enables ultra-low-power (ULP) DC lead-on detection. ECG lead-on
 * detection only functions when ECG is disabled.
 */

#define MAX86178_ECGLDCFG1_LON_EN_MASK  (1 << 7) /* Mask: ECG lead-on */
#define MAX86178_ECGLDCFG1_LON_DIS      (0 << 7) /* ECG DC lead-on disabled */
#define MAX86178_ECGLDCFG1_LON_EN       (1 << 7) /* ECG DC lead-on enabled */

/* This bit enables lead-off detection, which is either DC lead-off detection
 * or AC lead-off detection depending on OFFMODE. If ECG is not enabled,
 * this bit is ignored and lead-off detection is disabled.
 */

#define MAX86178_ECGLDCFG1_LOFF_EN_MASK (1 << 6) /* Mask: ECG lead-off */
#define MAX86178_ECGLDCFG1_LOFF_DIS     (0 << 6) /* ECG lead-off disabled */
#define MAX86178_ECGLDCFG1_LOFF_EN      (1 << 6) /* ECG lead-off enabled */

#define MAX86178_ECGLDCFG1_OFFMODE_MASK (1 << 3) /* Mask: ECG lead-off mode */
#define MAX86178_ECGLDCFG1_OFFMODE_DC   (0 << 3) /* DC lead-off mode */
#define MAX86178_ECGLDCFG1_OFFMODE_AC   (1 << 3) /* AC lead-off mode */

/* OFFFREQ selects the frequency divider of the square-wave stimulus for AC
 * lead-off, by dividing the ECG_ADC_CLK by N.
 */

#define MAX86178_ECGLDCFG1_OFFFREQ_MASK (7 << 0) /* Mask: ECG lead-off freq */
#define MAX86178_ECGLDCFG1_OFFFREQ_DC   (0 << 0) /* DC lead-off mode */
#define MAX86178_ECGLDCFG1_OFFFREQ_1    (1 << 0) /* Divider N = 1 */
#define MAX86178_ECGLDCFG1_OFFFREQ_2    (2 << 0) /* Divider N = 2 */
#define MAX86178_ECGLDCFG1_OFFFREQ_4    (3 << 0) /* Divider N = 4 */
#define MAX86178_ECGLDCFG1_OFFFREQ_8    (4 << 0) /* Divider N = 8 */
#define MAX86178_ECGLDCFG1_OFFFREQ_16   (5 << 0) /* Divider N = 16 */
#define MAX86178_ECGLDCFG1_OFFFREQ_32   (6 << 0) /* Divider N = 32 */
#define MAX86178_ECGLDCFG1_OFFFREQ_64   (7 << 0) /* Divider N = 64 */

/* ECGLDCFG2 bits masks and functions */

/* Select the current polarity for ECG DC lead-off detection.
 * 0： Non-inverted. ECGP sources current, ECGN sinks current.
 * 1： Inverted. ECGP sinks current, ECGN sources current.
 */

#define MAX86178_ECGLDCFG2_OFFIPOL_MASK (1 << 7) /* Mask: ECG lead-off IPOL */
#define MAX86178_ECGLDCFG2_OFFIPOL_NINV (0 << 7) /* Non-inverted, P -> N */
#define MAX86178_ECGLDCFG2_OFFIPOL_INV  (1 << 7) /* Inverted, N -> P */

/* Selects the DC/AC lead-off current amplitude */

#define MAX86178_ECGLDCFG2_OFFIMAG_MASK (7 << 4) /* Mask: LOFF magnitude */
#define MAX86178_ECGLDCFG2_OFFIMAG_DIS  (0 << 4) /* 0, sources are disabled */
#define MAX86178_ECGLDCFG2_OFFIMAG_5    (1 << 4) /* 5 nA DC / nAp-p AC */
#define MAX86178_ECGLDCFG2_OFFIMAG_10   (2 << 4) /* 10 nA DC / nAp-p AC */
#define MAX86178_ECGLDCFG2_OFFIMAG_20   (3 << 4) /* 20 nA DC / nAp-p AC */
#define MAX86178_ECGLDCFG2_OFFIMAG_50   (4 << 4) /* 50 nA DC / nAp-p AC */
#define MAX86178_ECGLDCFG2_OFFIMAG_100  (5 << 4) /* 100 nA DC / nAp-p AC */
#define MAX86178_ECGLDCFG2_OFFIMAG_200  (6 << 4) /* 200 nA DC / nAp-p AC */
#define MAX86178_ECGLDCFG2_OFFIMAG_400  (7 << 4) /* 400 nA DC / nAp-p AC */

/* THRES selects the voltage threshold for the DC or AC lead-off window
 * comparators, which are centered at VMID_ECG. If the voltage of either ECGP
 * or ECGN goes above the high threshold or below the low threshold, the
 * corresponding ECG_DC_LOFF status bit is set to 1 in register 0x03.
 */

#define MAX86178_ECGLDCFG2_THRES_MASK   0x0f      /* Mask: LOFF threshold */
#define MAX86178_ECGLDCFG2_THRES_25     1         /* VMID_ECG ± 25mV */
#define MAX86178_ECGLDCFG2_THRES_50     2         /* VMID_ECG ± 50mV */
#define MAX86178_ECGLDCFG2_THRES_75     3         /* VMID_ECG ± 75mV */
#define MAX86178_ECGLDCFG2_THRES_100    4         /* VMID_ECG ± 100mV */
#define MAX86178_ECGLDCFG2_THRES_125    4         /* VMID_ECG ± 125mV */
#define MAX86178_ECGLDCFG2_THRES_150    5         /* VMID_ECG ± 150mV */
#define MAX86178_ECGLDCFG2_THRES_175    6         /* VMID_ECG ± 175mV */
#define MAX86178_ECGLDCFG2_THRES_200    7         /* VMID_ECG ± 200mV */
#define MAX86178_ECGLDCFG2_THRES_225    8         /* VMID_ECG ± 225mV */
#define MAX86178_ECGLDCFG2_THRES_250    9         /* VMID_ECG ± 250mV */
#define MAX86178_ECGLDCFG2_THRES_275    10        /* VMID_ECG ± 275mV */
#define MAX86178_ECGLDCFG2_THRES_300    11        /* VMID_ECG ± 300mV */
#define MAX86178_ECGLDCFG2_THRES_325    12        /* VMID_ECG ± 325mV */
#define MAX86178_ECGLDCFG2_THRES_350    13        /* VMID_ECG ± 350mV */
#define MAX86178_ECGLDCFG2_THRES_375    14        /* VMID_ECG ± 375mV */
#define MAX86178_ECGLDCFG2_THRES_400    15        /* VMID_ECG ± 400mV*/

/* ECGLDBIAS1 bits masks and functions */

/* These bits select the ECG lead bias resistance, which is between ECGP and
 * VMID_ECG (ECGLDBIAS1_RP_EN), and ECGN and VMID_ECG (ECGLDBIAS1_RN_EN).
  */
#define MAX86178_ECGLDBIAS1_R_MASK      (3 << 2) /* Mask: bias resistance */
#define MAX86178_ECGLDBIAS1_R_50M       (0 << 2) /* R = 50 MOhm */
#define MAX86178_ECGLDBIAS1_R_100M      (1 << 2) /* R = 100 MOhm */
#define MAX86178_ECGLDBIAS1_R_200M      (2 << 2) /* R = 200 MOhm */

/* RP_EN/RN_EN enables the ECG lead bias between ECGP/ECGN and VMID_ECG with
 * the value selected by ECGLDIAS1_R.
 */
#define MAX86178_ECGLDBIAS1_RP_EN_MASK  (1 << 1) /* Mask: if enable P bias */
#define MAX86178_ECGLDBIAS1_RP_DIS      (0 << 1) /* No R bias */
#define MAX86178_ECGLDBIAS1_RP_EN       (1 << 1) /* ECGP --R bias--> VMID */

#define MAX86178_ECGLDBIAS1_RN_EN_MASK  (1 << 0) /* Mask: if enable N bias */
#define MAX86178_ECGLDBIAS1_RN_DIS      (0 << 0) /* No R bias */
#define MAX86178_ECGLDBIAS1_RN_EN       (1 << 0) /* ECGN --R bias--> VMID */

/* RLDCFG1 bits masks and functions */

/* Enable/disable the right leg drive circuit */

#define MAX86178_RLDCFG1_EN_MASK        (1 << 7) /* Mask: enable RLD */
#define MAX86178_RLDCFG1_DIS            (0 << 7) /* RLD circuit enabled */
#define MAX86178_RLDCFG1_EN             (1 << 7) /* RLD circuit disabled */

/* Control the RLD amplifier feedback switch used to control the RLD drive AC
 * feedback looP.
 * 0: Open-loop body-bias mode. The feedback network is shorted and the RLD
 *    amplifier acts as a DC buffer to bias the body to a voltage selected
 *    by BODY_BIAS.
 * 1: Closed-loop right leg drive mode. The RLD amplifier applies inverting
 *    gain to the AC common-mode input signal, forming a feedback loop
 *    through the body to bring the ECGP and ECGN inputs to the voltage
 *    selected by BODY_BIAS.
 */

#define MAX86178_RLDCFG1_MODE_MASK      (1 << 6) /* Mask: RLD mode */
#define MAX86178_RLDCFG1_MODE_OPEN      (0 << 6) /* RLD in open-loop mode */
#define MAX86178_RLDCFG1_MODE_CLOSE     (1 << 6) /* RLD in closed-loop mode */

/* RBIAS selects the lead-bias voltage for the ECG or BioZ inputs, which is
 * either the VMID_ECG reference or the output of the right leg drive common
 * mode averager (VRLD).
 */

#define MAX86178_RLDCFG1_RBIAS_MASK     (1 << 5) /* Mask: Lead-bias */
#define MAX86178_RLDCFG1_RBIAS_MID      (0 << 5) /* VLead-bias = VMID_ECG */
#define MAX86178_RLDCFG1_RBIAS_RLD      (1 << 5) /* VLead-bias = VRLD */

/* OOR_EN enabled the RLD out of range comparator, which sets the RLDOOR of
 * STAT4 status bit if the RLD amplifier output is below 0.127 x VAVDD or
 * above 0.870 x VAVDD.
 */

#define MAX86178_RLDCFG1_OOR_EN_MASK    (1 << 4) /* Mask: RLD_OOR enable */
#define MAX86178_RLDCFG1_OOR_DIS        (0 << 4) /* RLD OOR disabled */
#define MAX86178_RLDCFG1_OOR_EN         (1 << 4) /* RLD OOR enabled */

/* ACTV_CM_P/ACTV_CM_N enable the positive/negative input to the common-mode
 * averager. The positive/negative input is taken from either ECGP/ECGN or
 * CAPP/CAPN, selected by SEL_ECG[6](0x93). Enable both inputs for closed-
 * loop right leg applications.
 */

#define MAX86178_RLDCFG1_ACTV_CM_P_MASK (1 << 3) /* Mask */
#define MAX86178_RLDCFG1_ACTV_CM_P_DIS  (0 << 3) /* Positive input disabled */
#define MAX86178_RLDCFG1_ACTV_CM_P_EN   (1 << 3) /* Positive input enabled */

#define MAX86178_RLDCFG1_ACTV_CM_N_MASK (1 << 2) /* Mask */
#define MAX86178_RLDCFG1_ACTV_CM_N_DIS  (0 << 2) /* Negative input disabled */
#define MAX86178_RLDCFG1_ACTV_CM_N_EN   (1 << 2) /* Negative input enabled */

/* Select the internal RLD gain when RLDCFG2_EXT_RES is set to 0 and
 * RLDCFG1_MODE is set to 1.
 */

#define MAX86178_RLDCFG1_GAIN_MASK      (3 << 0) /* Mask: RLD gain */
#define MAX86178_RLDCFG1_GAIN_12        (0 << 0) /* Gain = 12 V/V */
#define MAX86178_RLDCFG1_GAIN_24        (1 << 0) /* Gain = 24 V/V */
#define MAX86178_RLDCFG1_GAIN_48        (2 << 0) /* Gain = 48 V/V */
#define MAX86178_RLDCFG1_GAIN_97        (3 << 0) /* Gain = 97 V/V */

/* RLDCFG2 bits masks and functions */

/* EXT_RES disconnects the internal feedback resistor to use an external
 * gain-setting resistor. EXT_RES is effective only when RLDCFG1_MODE is set
 * to 1. The gain setting resistor RRLDFB must be connected between the RLD
 * and RLD_INV pins, which each have an internal 50kΩ series resistor. The
 * resulting gain is (100kΩ + RRLDFB) / 150kΩ when both common-mode
 * averager inputs are enabled, and (100kΩ + RRLDFB) / 250kΩ when only one
 * is enabled. When it's set to 0, internal resistor and external resistor
 * (if connected) are in parallel.
 */

#define MAX86178_RLDCFG2_EXT_RES_MASK   (1 << 7) /* Mask: External resistor */
#define MAX86178_RLDCFG2_INT_RES        (0 << 7) /* Internal R enabled */
#define MAX86178_RLDCFG2_EXT_RES        (1 << 7) /* Internal R disconnected */

/* SEL_ECG selects ECGP/ECGN or CAPP/CAPN as the inputs to the RLD common-
 * mode averager. The common-mode averager input can be switched to the BioZ
 * inputs by setting BIOZCFG8_RLD_SEL to 1. The voltages at CAPP/CAPN are
 * buffered by the ECG INA, so their common-mode voltage lags slightly behind
 * the ECGP/ECGN common-mode voltage. This introduces phase lag into the RLD
 * feedback loop resulting in less common-mode signal attenuation.
 *  BIOZCFG8_RLD_SEL  |  RLDCFG2_SEL_ECG  |  RLD input selection
 *         0          |         0         |      CAPP/CAPN
 *         0          |         1         |      ECGP/ECGN
 *         1          |         x         |       BIP/BIN
 */

#define MAX86178_RLDCFG2_SEL_ECG_MASK   (1 << 6) /* Mask: External resistor */
#define MAX86178_RLDCFG2_SEL_CAP        (0 << 6) /* See the table above */
#define MAX86178_RLDCFG2_SEL_ECG        (1 << 6) /* See the table above */

/* Select the bandwidth for the right leg drive amplifier:
 * RLDCFG2_BW | RLD gain = 12 | RLD gain = 24 | RLD gain = 48 | RLD gain = 97
 *     0      |    917 Hz     |    697 Hz     |    647 Hz     |    555 Hz
 *     1      |    937 Hz     |    724 Hz     |    698 Hz     |    648 Hz
 *     2      |    944 Hz     |    733 Hz     |    716 Hz     |    681 Hz
 *     3      |    948 Hz     |    736 Hz     |    724 Hz     |    696 Hz
 */

#define MAX86178_RLDCFG2_BW_MASK        (3 << 4) /* Mask: RLD bandwidth */
#define MAX86178_RLDCFG2_BW_0           (0 << 4) /* See the table above */
#define MAX86178_RLDCFG2_BW_1           (1 << 4) /* See the table above */
#define MAX86178_RLDCFG2_BW_2           (2 << 4) /* See the table above */
#define MAX86178_RLDCFG2_BW_3           (3 << 4) /* See the table above */

/* BODY_BIAS sets the voltage at the noninverting terminal of the RLD
 * amplifier. The voltage is VMID_ECG - (BODY_BIAS_DAC x 0.04V), where
 * VMID_ECG is 0.76V (typ). BODY_BIAS is a 2's complement representation.
 */

#define MAX86178_RLDCFG2_BODY_BIAS_MASK 0x0f     /* Mask: Body bias */
#define MAX86178_RLDCFG2_BODY_BIAS_0p76 0        /* VMID_ECG = 0.76V (mid) */
#define MAX86178_RLDCFG2_BODY_BIAS_0p72 1        /* VMID_ECG = 0.72V */
#define MAX86178_RLDCFG2_BODY_BIAS_0p68 2        /* VMID_ECG = 0.68V */
#define MAX86178_RLDCFG2_BODY_BIAS_0p64 3        /* VMID_ECG = 0.64V */
#define MAX86178_RLDCFG2_BODY_BIAS_0p60 4        /* VMID_ECG = 0.60V */
#define MAX86178_RLDCFG2_BODY_BIAS_0p56 5        /* VMID_ECG = 0.56V */
#define MAX86178_RLDCFG2_BODY_BIAS_0p52 6        /* VMID_ECG = 0.52V */
#define MAX86178_RLDCFG2_BODY_BIAS_0p48 7        /* VMID_ECG = 0.48V (min) */
#define MAX86178_RLDCFG2_BODY_BIAS_1p08 8        /* VMID_ECG = 1.08V (max) */
#define MAX86178_RLDCFG2_BODY_BIAS_1p04 9        /* VMID_ECG = 1.04V */
#define MAX86178_RLDCFG2_BODY_BIAS_1p00 10       /* VMID_ECG = 1.00V */
#define MAX86178_RLDCFG2_BODY_BIAS_0p96 11       /* VMID_ECG = 0.96V */
#define MAX86178_RLDCFG2_BODY_BIAS_0p92 12       /* VMID_ECG = 0.92V */
#define MAX86178_RLDCFG2_BODY_BIAS_0p88 13       /* VMID_ECG = 0.88V */
#define MAX86178_RLDCFG2_BODY_BIAS_0p84 14       /* VMID_ECG = 0.84V */
#define MAX86178_RLDCFG2_BODY_BIAS_0p80 15       /* VMID_ECG = 0.80V */

/* RESPCFG1 bits masks and functions */

/* LPFDUTY sets the duty cycle of the respiration current generator common-
 * mode-feedback low-pass filter. Doubling the duty cycle doubles the common-
 * mode-feedback bandwidth. Higher bandwidths result in faster current source
 * settling, while lower bandwidths maintain higher current source output
 * impedance. The aprroximate bandwidthes are:
 *   LPFDUTY  |  tHIGH (BIOZ_SYNTH_CLK cycles)  | approximate bandwidth
 *      0     |               1                 |        0.98 Hz
 *      0     |               2                 |        1.95 Hz
 *      0     |               4                 |        3.90 Hz
 *      0     |               8                 |        7.79 Hz
 *      0     |               16                |        15.54 Hz
 *      0     |               32                |        30.89 Hz
 *      0     |               64                |        61.08 Hz
 *      0     |            Always On            |          N/A
 */

#define MAX86178_RESPCFG1_LPFDUTY_MASK  (7 << 5) /* Mask */
#define MAX86178_RESPCFG1_LPFDUTY_1     (0 << 5) /* See the table above */
#define MAX86178_RESPCFG1_LPFDUTY_2     (1 << 5) /* See the table above */
#define MAX86178_RESPCFG1_LPFDUTY_4     (2 << 5) /* See the table above */
#define MAX86178_RESPCFG1_LPFDUTY_8     (3 << 5) /* See the table above */
#define MAX86178_RESPCFG1_LPFDUTY_16    (4 << 5) /* See the table above */
#define MAX86178_RESPCFG1_LPFDUTY_32    (5 << 5) /* See the table above */
#define MAX86178_RESPCFG1_LPFDUTY_64    (6 << 5) /* See the table above */
#define MAX86178_RESPCFG1_LPFDUTY_ON    (7 << 5) /* See the table above */

/* CHOPCLK selects the respiration current source chopping clock divider
 * ratio from BIOZ_SYNTH_CLK. The respiration current generators are chopped
 * at a frequency of BIOZ_SYNTH_CLK / CHOPCLK.
 */

#define MAX86178_RESPCFG1_CHOPCLK_MASK  (3 << 3) /* Mask: chop clock */
#define MAX86178_RESPCFG1_CHOPCLK_256   (0 << 3) /* Divider ration = 256 */
#define MAX86178_RESPCFG1_CHOPCLK_512   (1 << 3) /* Divider ration = 512 */
#define MAX86178_RESPCFG1_CHOPCLK_1024  (2 << 3) /* Divider ration = 1024 */
#define MAX86178_RESPCFG1_CHOPCLK_2048  (3 << 3) /* Divider ration = 2048 */

/* MODE selects a current source dynamic matching and common-mode feedback
 * scheme for the respiration-current generators. These schemes can help
 * mitigate flicker noise in the respiration current generators.
 * 0: Dynamic matching disabled with analog low-pass filter.
 * 1: Dynamic matching enabled without analog low-pass filter.
 * 2: Dynamic matching enabled with analog low-pass filter.
 * 3: Dynamic matching enabled with internal resistive-common mode load.
 */

#define MAX86178_RESPCFG1_MODE_MASK     (3 << 1) /* Mask */
#define MAX86178_RESPCFG1_MODE_ALPF     (0 << 1) /* See the comment above */
#define MAX86178_RESPCFG1_MODE_DMATCH   (1 << 1) /* See the comment above */
#define MAX86178_RESPCFG1_MODE_DMLPF    (2 << 1) /* See the comment above */
#define MAX86178_RESPCFG1_MODE_DMCM     (3 << 1) /* See the comment above */

/* When the BioZ channel is enabled, this bit switches the stimulus source
 * from the transmit channel to the balanced respiration current generators.
 * BIOZCFG1_BG_EN must be set to 1 and BIOZCFG1_EN must be set to 0x1 or 0x2.
 */

#define MAX86178_RESPCFG1_EN_MASK       (1 << 0) /* Mask */
#define MAX86178_RESPCFG1_DIS           (0 << 0) /* Respiration disabled */
#define MAX86178_RESPCFG1_EN            (1 << 0) /* Respiration enabled */

/* INTxEN1 bits masks and functions */

/* Enables the STAT1_* bit to trigger the INTx output pin. */

#define MAX86178_INTXEN1_AFULL_EN       (1 << 7) /* STAT1_FIFOAFULL */
#define MAX86178_INTXEN1_FRAMERDY_EN    (1 << 6) /* STAT1_PPGFRAMERDY */
#define MAX86178_INTXEN1_FIFORDY_EN     (1 << 5) /* STAT1_FIFODATARDY */
#define MAX86178_INTXEN1_ALC_OVF_EN     (1 << 4) /* STAT1_ALC_OVF */
#define MAX86178_INTXEN1_EXP_OVF_EN     (1 << 3) /* STAT1_EXP_OVF */
#define MAX86178_INTXEN1_PPGTHRES2_EN   (1 << 2) /* STAT1_PPGTHRES2HILO */
#define MAX86178_INTXEN1_PPGTHRES1_EN   (1 << 1) /* STAT1_PPGTHRES1HILO */

/* INTxEN2 bits masks and functions */

/* Enables the STAT2_* bit to trigger the INTx output pin. */

#define MAX86178_INTXEN2_INVLDPPG_EN    (1 << 7) /* STAT2_INVLDPPGCFG */
#define MAX86178_INTXEN2_LED6COMPB_EN   (1 << 5) /* STAT2_LED6COMPB */
#define MAX86178_INTXEN2_LED5COMPB_EN   (1 << 4) /* STAT2_LED5COMPB */
#define MAX86178_INTXEN2_LED4COMPB_EN   (1 << 3) /* STAT2_LED4COMPB */
#define MAX86178_INTXEN2_LED3COMPB_EN   (1 << 2) /* STAT2_LED3COMPB */
#define MAX86178_INTXEN2_LED2COMPB_EN   (1 << 1) /* STAT2_LED2COMPB */
#define MAX86178_INTXEN2_LED1COMPB_EN   (1 << 0) /* STAT2_LED1COMPB */

/* INTxEN3 bits masks and functions */

/* Enables the STAT3_* bit to trigger the INTx output pin. */

#define MAX86178_INTXEN3_FREQUNLOCK_EN  (1 << 4) /* STAT3_FREQ_UNLOCK */
#define MAX86178_INTXEN3_FREQLOCK_EN    (1 << 3) /* STAT3_FREQ_LOCK */
#define MAX86178_INTXEN3_PHASEUNLOCK_EN (1 << 2) /* STAT3_PHASE_UNLOCK */
#define MAX86178_INTXEN3_PHASELOCK_EN   (1 << 1) /* STAT3_PHASE_LOCK */

/* INTxEN4 bits masks and functions */

/* Enables the STAT4_* bit to trigger the INTx output pin. */

#define MAX86178_INTXEN4_ECG_LON_EN     (1 << 7) /* STAT4_ECG_LON */
#define MAX86178_INTXEN4_ECG_FASTREC_EN (1 << 5) /* STAT4_ECG_FASTREC */
#define MAX86178_INTXEN4_ECG_RLDOOR_EN  (1 << 4) /* STAT4_ECG_RLDOOR */
#define MAX86178_INTXEN4_ECGP_LOFFH_EN  (1 << 3) /* STAT4_ECGP_LOFFH */
#define MAX86178_INTXEN4_ECGP_LOFFL_EN  (1 << 2) /* STAT4_ECGP_LOFFL */
#define MAX86178_INTXEN4_ECGN_LOFFH_EN  (1 << 1) /* STAT4_ECGN_LOFFH */
#define MAX86178_INTXEN4_ECGN_LOFFL_EN  (1 << 0) /* STAT4_ECGN_LOFFL */

/* INTxEN5 bits masks and functions */

/* Enables the STAT5_* bit to trigger the INTx output pin. */

#define MAX86178_INTXEN5_BIOZ_LON_EN    (1 << 7) /* STAT5_BIOZ_LON */
#define MAX86178_INTXEN5_BIOZ_OVER_EN   (1 << 6) /* STAT5_BIOZ_OVER */
#define MAX86178_INTXEN5_BIOZ_UNDR_EN   (1 << 5) /* STAT5_BIOZ_UNDR */
#define MAX86178_INTXEN5_BIOZ_DRVOOR_EN (1 << 4) /* STAT5_BIOZ_DRVOOR */
#define MAX86178_INTXEN5_BIP_LOFFH      (1 << 3) /* STAT5_BIP_LOFFH */
#define MAX86178_INTXEN5_BIP_LOFFL      (1 << 2) /* STAT5_BIP_LOFFL */
#define MAX86178_INTXEN5_BIN_LOFFH      (1 << 1) /* STAT5_BIN_LOFFH */
#define MAX86178_INTXEN5_BIN_LOFFL      (1 << 0) /* STAT5_BIN_LOFFL */

/* Default, read-only, or constant values of registers */

/* Each sample in FIFO has 3 bytes and start with pre-defined tags. Most of
 * them can be distinguished from MSB 4bits, some of them need post-
 * processing according to MSB 6bits. Besides, there are four kinds of
 * special tags counts from 0xfffffc.
 */

#define MAX86178_FIFOTAG_MASK_PRE       0xf00000   /* MSB 4bits mask */
#define MAX86178_FIFOTAG_MASK_POST      0xfc0000   /* MSB 6bits mask */
#define MAX86178_FIFOTAG_PRE_MEAS1      (0 << 20)  /* MEAS1 sample */
#define MAX86178_FIFOTAG_PRE_MEAS2      (1 << 20)  /* MEAS2 sample */
#define MAX86178_FIFOTAG_PRE_MEAS3      (2 << 20)  /* MEAS3 sample */
#define MAX86178_FIFOTAG_PRE_MEAS4      (3 << 20)  /* MEAS4 sample */
#define MAX86178_FIFOTAG_PRE_MEAS5      (4 << 20)  /* MEAS5 sample */
#define MAX86178_FIFOTAG_PRE_MEAS6      (5 << 20)  /* MEAS6 sample */
#define MAX86178_FIFOTAG_PRE_DARK       (6 << 20)  /* Dark sample */
#define MAX86178_FIFOTAG_PRE_ALC_OVF    (7 << 20)  /* Dark overflow */
#define MAX86178_FIFOTAG_PRE_EXP_OVF    (8 << 20)  /* Exposure overflow */
#define MAX86178_FIFOTAG_PRE_BIOZI      (9 << 20)  /* BioZ I channel data */
#define MAX86178_FIFOTAG_PRE_BIOZQ      (10 << 20) /* BioZ Q channel data */
#define MAX86178_FIFOTAG_PRE_ECG        (11 << 20) /* ECG sample */
#define MAX86178_FIFOTAG_PRE_ECG_DIF    (12 << 20) /* ECGP, ECGN, or diff */
#define MAX86178_FIFOTAG_PRE_CAP_DIF    (13 << 20) /* CAPP, CAPN, or diff */
#define MAX86178_FIFOTAG_PRE_TIMING     (14 << 20) /* Timing data */
#define MAX86178_FIFOTAG_PRE_SPECIAL    (15 << 20) /* Special TAGs */
#define MAX86178_FIFOTAG_POST_ECG       0xb00000   /* Normal ECG sample */
#define MAX86178_FIFOTAG_POST_ECG_REC   0xb40000   /* ECG in fast recovery */
#define MAX86178_FIFOTAG_POST_EPTIMING  0xe00000   /* ECG to PPG timing */
#define MAX86178_FIFOTAG_POST_BPTIMING  0xe40000   /* BioZ to PPG timing */
#define MAX86178_FIFOTAG_POST_EBTIMING  0xe80000   /* ECG to BioZ timing */
#define MAX86178_FIFOTAG_MWBA_ENTER     0xfffffc   /* Enter MWBA mode */
#define MAX86178_FIFOTAG_MWBA_EXIT      0xfffffd   /* Exit MWBA mode */
#define MAX86178_FIFOTAG_MARKER         0xfffffe   /* Custom marker */
#define MAX86178_FIFOTAG_INVALID        0xffffff   /* Invalid data */

#define MAX86178_ECG_SIGN_MASK          0x020000   /* Sign of ECG value */
#define MAX86178_ECG_DEC_RATE_16        16         /* Decimation ratio: 16 */
#define MAX86178_ECG_DEC_RATE_32        32         /* Decimation ratio: 32 */
#define MAX86178_ECG_DEC_RATE_64        64         /* Decimation ratio: 64 */
#define MAX86178_ECG_DEC_RATE_128       128        /* Decimation ratio: 128 */
#define MAX86178_ECG_DEC_RATE_256       256        /* Decimation ratio: 256 */
#define MAX86178_ECG_DEC_RATE_512       512        /* Decimation ratio: 512 */

#define MAX86178_PARTID                 0x43       /* Part identifier */

/* Native attributes of registers */

#define MAX86178_FIFO_BYTES_PER_SAMPLE  3          /* Bytes/data in FIFO */
#define MAX86178_FIFO_SIZE_SAMPLES      256        /* Max samples in FIFO */
#define MAX86178_FIFO_SIZE_BYTES        768        /* Max bytes in FIFO */
#define MAX86178_PLL_FREQ_LOCK_TIME     4          /* Unit in ms (max.) */
#define MAX86178_PLL_PHASE_LOCK_TIME    9          /* Unit in ms (max.) */
#define MAX86178_FRCLKDIV_MAX           32766      /* Max. FR_CLK_DIV */
#define MAX86178_FRCLKDIV_MIN           16         /* Min. FR_CLK_DIV */

/* Absolute parameters */

#define MAX86178_ABS_PPG_MAX            0x0fffff   /* Max PPG value */

/****************************************************************************
 * Public Types
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
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_MAX86178 */
#endif /* __INCLUDE_NUTTX_SENSORS_MAX86178_REG_H */

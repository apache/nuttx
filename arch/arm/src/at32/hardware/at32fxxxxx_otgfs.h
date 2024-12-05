/****************************************************************************
 * arch/arm/src/at32/hardware/at32fxxxxx_otgfs.h
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

#ifndef __ARCH_ARM_SRC_AT32_HARDWARE_AT32FXXXXX_OTGFS_H
#define __ARCH_ARM_SRC_AT32_HARDWARE_AT32FXXXXX_OTGFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* General definitions */

#define OTGFS_EPTYPE_CTRL               (0) /* Control */
#define OTGFS_EPTYPE_ISOC               (1) /* Isochronous */
#define OTGFS_EPTYPE_BULK               (2) /* Bulk */
#define OTGFS_EPTYPE_INTR               (3) /* Interrupt */

#define OTGFS_PID_DATA0                 (0)
#define OTGFS_PID_DATA2                 (1)
#define OTGFS_PID_DATA1                 (2)
#define OTGFS_PID_MDATA                 (3) /* Non-control */
#define OTGFS_PID_SETUP                 (3) /* Control */

/* Register Offsets *********************************************************/

/* Core global control and status registers */

#define AT32_OTGFS_GOTGCTL_OFFSET      0x0000 /* Control and status register */
#define AT32_OTGFS_GOTGINT_OFFSET      0x0004 /* Interrupt register */
#define AT32_OTGFS_GAHBCFG_OFFSET      0x0008 /* AHB configuration register */
#define AT32_OTGFS_GUSBCFG_OFFSET      0x000c /* USB configuration register */
#define AT32_OTGFS_GRSTCTL_OFFSET      0x0010 /* Reset register */
#define AT32_OTGFS_GINTSTS_OFFSET      0x0014 /* Core interrupt register */
#define AT32_OTGFS_GINTMSK_OFFSET      0x0018 /* Interrupt mask register */
#define AT32_OTGFS_GRXSTSR_OFFSET      0x001c /* Receive status debug read/OTG status read register */
#define AT32_OTGFS_GRXSTSP_OFFSET      0x0020 /* Receive status debug read/OTG status pop register */
#define AT32_OTGFS_GRXFSIZ_OFFSET      0x0024 /* Receive FIFO size register */
#define AT32_OTGFS_HNPTXFSIZ_OFFSET    0x0028 /* Host non-periodic transmit FIFO size register */
#define AT32_OTGFS_DIEPTXF0_OFFSET     0x0028 /* Endpoint 0 Transmit FIFO size */
#define AT32_OTGFS_HNPTXSTS_OFFSET     0x002c /* Non-periodic transmit FIFO/queue status register */
#define AT32_OTGFS_GCCFG_OFFSET        0x0038 /* General core configuration register */
#define AT32_OTGFS_CID_OFFSET          0x003c /* Core ID register  */
#define AT32_OTGFS_HPTXFSIZ_OFFSET     0x0100 /* Host periodic transmit FIFO size register */

#define AT32_OTGFS_DIEPTXF_OFFSET(n)   (0x0104+(((n)-1) << 2)) /* n: 1 ~ 7 */

#define AT32_OTGFS_DIEPTXF1_OFFSET     0x0104 /* Device IN endpoint transmit FIFO1 size register */
#define AT32_OTGFS_DIEPTXF2_OFFSET     0x0108 /* Device IN endpoint transmit FIFO2 size register */
#define AT32_OTGFS_DIEPTXF3_OFFSET     0x010c /* Device IN endpoint transmit FIFO3 size register */
#define AT32_OTGFS_DIEPTXF4_OFFSET     0x0110 /* Device IN endpoint transmit FIFO4 size register */
#define AT32_OTGFS_DIEPTXF5_OFFSET     0x0114 /* Device IN endpoint transmit FIFO5 size register */
#define AT32_OTGFS_DIEPTXF6_OFFSET     0x0118 /* Device IN endpoint transmit FIFO6 size register */
#define AT32_OTGFS_DIEPTXF7_OFFSET     0x011c /* Device IN endpoint transmit FIFO7 size register */

/* Host-mode control and status registers */

#define AT32_OTGFS_HCFG_OFFSET         0x0400 /* Host configuration register */
#define AT32_OTGFS_HFIR_OFFSET         0x0404 /* Host frame interval register */
#define AT32_OTGFS_HFNUM_OFFSET        0x0408 /* Host frame number/frame time remaining register */
#define AT32_OTGFS_HPTXSTS_OFFSET      0x0410 /* Host periodic transmit FIFO/queue status register */
#define AT32_OTGFS_HAINT_OFFSET        0x0414 /* Host all channels interrupt register */
#define AT32_OTGFS_HAINTMSK_OFFSET     0x0418 /* Host all channels interrupt mask register */
#define AT32_OTGFS_HPRT_OFFSET         0x0440 /* Host port control and status register */

#define AT32_OTGFS_CHAN_OFFSET(n)      (0x500 + ((n) << 5) /* n: 0 ~ 15 */

#define AT32_OTGFS_HCCHAR_CHOFFSET     0x0000 /* Host channel characteristics register */
#define AT32_OTGFS_HCINT_CHOFFSET      0x0008 /* Host channel interrupt register */
#define AT32_OTGFS_HCINTMSK_CHOFFSET   0x000c /* Host channel interrupt mask register */
#define AT32_OTGFS_HCTSIZ_CHOFFSET     0x0010 /* Host channel interrupt register */

#define AT32_OTGFS_HCCHAR_OFFSET(n)    (0x500 + ((n) << 5)) /* n: 0 ~ 15 */

#define AT32_OTGFS_HCCHAR0_OFFSET      0x0500 /* Host channel-0 characteristics register */
#define AT32_OTGFS_HCCHAR1_OFFSET      0x0520 /* Host channel-1 characteristics register */
#define AT32_OTGFS_HCCHAR2_OFFSET      0x0540 /* Host channel-2 characteristics register */
#define AT32_OTGFS_HCCHAR3_OFFSET      0x0560 /* Host channel-3 characteristics register */
#define AT32_OTGFS_HCCHAR4_OFFSET      0x0580 /* Host channel-4 characteristics register */
#define AT32_OTGFS_HCCHAR5_OFFSET      0x05a0 /* Host channel-5 characteristics register */
#define AT32_OTGFS_HCCHAR6_OFFSET      0x05c0 /* Host channel-6 characteristics register */
#define AT32_OTGFS_HCCHAR7_OFFSET      0x05e0 /* Host channel-7 characteristics register */
#define AT32_OTGFS_HCCHAR8_OFFSET      0x0600 /* Host channel-8 characteristics register */
#define AT32_OTGFS_HCCHAR9_OFFSET      0x0620 /* Host channel-9 characteristics register */
#define AT32_OTGFS_HCCHAR10_OFFSET     0x060  /* Host channel-10 characteristics register */
#define AT32_OTGFS_HCCHAR11_OFFSET     0x0660 /* Host channel-11 characteristics register */
#define AT32_OTGFS_HCCHAR12_OFFSET     0x0680 /* Host channel-12 characteristics register */
#define AT32_OTGFS_HCCHAR13_OFFSET     0x06a0 /* Host channel-13 characteristics register */
#define AT32_OTGFS_HCCHAR14_OFFSET     0x06c0 /* Host channel-14 characteristics register */
#define AT32_OTGFS_HCCHAR15_OFFSET     0x06e0 /* Host channel-15 characteristics register */

#define AT32_OTGFS_HCINT_OFFSET(n)     (0x508 + ((n) << 5)) /* n: 0 ~ 15 */

#define AT32_OTGFS_HCINT0_OFFSET       0x0508 /* Host channel-0 interrupt register */
#define AT32_OTGFS_HCINT1_OFFSET       0x0528 /* Host channel-1 interrupt register */
#define AT32_OTGFS_HCINT2_OFFSET       0x0548 /* Host channel-2 interrupt register */
#define AT32_OTGFS_HCINT3_OFFSET       0x0568 /* Host channel-3 interrupt register */
#define AT32_OTGFS_HCINT4_OFFSET       0x0588 /* Host channel-4 interrupt register */
#define AT32_OTGFS_HCINT5_OFFSET       0x05a8 /* Host channel-5 interrupt register */
#define AT32_OTGFS_HCINT6_OFFSET       0x05c8 /* Host channel-6 interrupt register */
#define AT32_OTGFS_HCINT7_OFFSET       0x05e8 /* Host channel-7 interrupt register */
#define AT32_OTGFS_HCINT8_OFFSET       0x0608 /* Host channel-8 interrupt register */
#define AT32_OTGFS_HCINT9_OFFSET       0x0628 /* Host channel-9 interrupt register */
#define AT32_OTGFS_HCINT10_OFFSET      0x0648 /* Host channel-10 interrupt register */
#define AT32_OTGFS_HCINT11_OFFSET      0x0668 /* Host channel-11 interrupt register */
#define AT32_OTGFS_HCINT12_OFFSET      0x0688 /* Host channel-12 interrupt register */
#define AT32_OTGFS_HCINT13_OFFSET      0x06a8 /* Host channel-13 interrupt register */
#define AT32_OTGFS_HCINT14_OFFSET      0x06c8 /* Host channel-14 interrupt register */
#define AT32_OTGFS_HCINT15_OFFSET      0x06e8 /* Host channel-15 interrupt register */

#define AT32_OTGFS_HCINTMSK_OFFSET(n)  (0x50c + ((n) << 5)) /* n: 0 ~ 15 */

#define AT32_OTGFS_HCINTMSK0_OFFSET    0x050c /* Host channel-0 interrupt mask register */
#define AT32_OTGFS_HCINTMSK1_OFFSET    0x052c /* Host channel-1 interrupt mask register */
#define AT32_OTGFS_HCINTMSK2_OFFSET    0x054c /* Host channel-2 interrupt mask register */
#define AT32_OTGFS_HCINTMSK3_OFFSET    0x056c /* Host channel-3 interrupt mask register */
#define AT32_OTGFS_HCINTMSK4_OFFSET    0x058c /* Host channel-4 interrupt mask register */
#define AT32_OTGFS_HCINTMSK5_OFFSET    0x05ac /* Host channel-5 interrupt mask register */
#define AT32_OTGFS_HCINTMSK6_OFFSET    0x05cc /* Host channel-6 interrupt mask register */
#define AT32_OTGFS_HCINTMSK7_OFFSET    0x05ec /* Host channel-7 interrupt mask register */
#define AT32_OTGFS_HCINTMSK8_OFFSET    0x060c /* Host channel-8 interrupt mask register */
#define AT32_OTGFS_HCINTMSK9_OFFSET    0x062c /* Host channel-9 interrupt mask register */
#define AT32_OTGFS_HCINTMSK10_OFFSET   0x064c /* Host channel-10 interrupt mask register */
#define AT32_OTGFS_HCINTMSK11_OFFSET   0x066c /* Host channel-11 interrupt mask register */
#define AT32_OTGFS_HCINTMSK12_OFFSET   0x068c /* Host channel-12 interrupt mask register */
#define AT32_OTGFS_HCINTMSK13_OFFSET   0x06ac /* Host channel-13 interrupt mask register */
#define AT32_OTGFS_HCINTMSK14_OFFSET   0x06cc /* Host channel-14 interrupt mask register */
#define AT32_OTGFS_HCINTMSK15_OFFSET   0x06ec /* Host channel-15 interrupt mask register */

#define AT32_OTGFS_HCTSIZ_OFFSET(n)    (0x510 + ((n) << 5)) /* n: 0 ~ 15 */

#define AT32_OTGFS_HCTSIZ0_OFFSET      0x0510 /* Host channel-0 interrupt register */
#define AT32_OTGFS_HCTSIZ1_OFFSET      0x0530 /* Host channel-1 interrupt register */
#define AT32_OTGFS_HCTSIZ2_OFFSET      0x0550 /* Host channel-2 interrupt register */
#define AT32_OTGFS_HCTSIZ3_OFFSET      0x0570 /* Host channel-3 interrupt register */
#define AT32_OTGFS_HCTSIZ4_OFFSET      0x0590 /* Host channel-4 interrupt register */
#define AT32_OTGFS_HCTSIZ5_OFFSET      0x05b0 /* Host channel-5 interrupt register */
#define AT32_OTGFS_HCTSIZ6_OFFSET      0x05d0 /* Host channel-6 interrupt register */
#define AT32_OTGFS_HCTSIZ7_OFFSET      0x05f0 /* Host channel-7 interrupt register */
#define AT32_OTGFS_HCTSIZ8_OFFSET      0x0610 /* Host channel-8 interrupt register */
#define AT32_OTGFS_HCTSIZ9_OFFSET      0x0630 /* Host channel-9 interrupt register */
#define AT32_OTGFS_HCTSIZ10_OFFSET     0x0650 /* Host channel-10 interrupt register */
#define AT32_OTGFS_HCTSIZ11_OFFSET     0x0670 /* Host channel-11 interrupt register */
#define AT32_OTGFS_HCTSIZ12_OFFSET     0x0690 /* Host channel-12 interrupt register */
#define AT32_OTGFS_HCTSIZ13_OFFSET     0x06b0 /* Host channel-13 interrupt register */
#define AT32_OTGFS_HCTSIZ14_OFFSET     0x06d0 /* Host channel-14 interrupt register */
#define AT32_OTGFS_HCTSIZ15_OFFSET     0x06f0 /* Host channel-15 interrupt register */

/* Device-mode control and status registers */

#define AT32_OTGFS_DCFG_OFFSET         0x0800 /* Device configuration register */
#define AT32_OTGFS_DCTL_OFFSET         0x0804 /* Device control register */
#define AT32_OTGFS_DSTS_OFFSET         0x0808 /* Device status register */
#define AT32_OTGFS_DIEPMSK_OFFSET      0x0810 /* Device IN endpoint common interrupt mask register */
#define AT32_OTGFS_DOEPMSK_OFFSET      0x0814 /* Device OUT endpoint common interrupt mask register */
#define AT32_OTGFS_DAINT_OFFSET        0x0818 /* Device all endpoints interrupt register */
#define AT32_OTGFS_DAINTMSK_OFFSET     0x081c /* All endpoints interrupt mask register */
#define AT32_OTGFS_DIEPEMPMSK_OFFSET   0x0834 /* Device IN endpoint FIFO empty interrupt mask register */

#define AT32_OTGFS_DIEP_OFFSET(n)      (0x0900 + ((n) << 5)) /* n: 0 ~ 7 */

#define AT32_OTGFS_DIEPCTL_EPOFFSET    0x0000 /* Device endpoint control register */
#define AT32_OTGFS_DIEPINT_EPOFFSET    0x0008 /* Device endpoint interrupt register */
#define AT32_OTGFS_DIEPTSIZ_EPOFFSET   0x0010 /* Device IN endpoint transfer size register */
#define AT32_OTGFS_DTXFSTS_EPOFFSET    0x0018 /* Device IN endpoint transmit FIFO status register */

#define AT32_OTGFS_DIEPCTL_OFFSET(n)   (0x0900 + ((n) << 5)) /* n: 0 ~ 7 */

#define AT32_OTGFS_DIEPCTL0_OFFSET     0x0900 /* Device control IN endpoint 0 control register */
#define AT32_OTGFS_DIEPCTL1_OFFSET     0x0920 /* Device control IN endpoint 1 control register */
#define AT32_OTGFS_DIEPCTL2_OFFSET     0x0940 /* Device control IN endpoint 2 control register */
#define AT32_OTGFS_DIEPCTL3_OFFSET     0x0960 /* Device control IN endpoint 3 control register */
#define AT32_OTGFS_DIEPCTL4_OFFSET     0x0980 /* Device control IN endpoint 4 control register */
#define AT32_OTGFS_DIEPCTL5_OFFSET     0x09a0 /* Device control IN endpoint 5 control register */
#define AT32_OTGFS_DIEPCTL6_OFFSET     0x09c0 /* Device control IN endpoint 6 control register */
#define AT32_OTGFS_DIEPCTL7_OFFSET     0x09e0 /* Device control IN endpoint 7 control register */

#define AT32_OTGFS_DIEPINT_OFFSET(n)   (0x0908 + ((n) << 5)) /* n: 0 ~ 7 */

#define AT32_OTGFS_DIEPINT0_OFFSET     0x0908 /* Device endpoint-0 interrupt register */
#define AT32_OTGFS_DIEPINT1_OFFSET     0x0928 /* Device endpoint-1 interrupt register */
#define AT32_OTGFS_DIEPINT2_OFFSET     0x0948 /* Device endpoint-2 interrupt register */
#define AT32_OTGFS_DIEPINT3_OFFSET     0x0968 /* Device endpoint-3 interrupt register */
#define AT32_OTGFS_DIEPINT4_OFFSET     0x0988 /* Device endpoint-4 interrupt register */
#define AT32_OTGFS_DIEPINT5_OFFSET     0x09a8 /* Device endpoint-5 interrupt register */
#define AT32_OTGFS_DIEPINT6_OFFSET     0x09c8 /* Device endpoint-6 interrupt register */
#define AT32_OTGFS_DIEPINT7_OFFSET     0x09e8 /* Device endpoint-7 interrupt register */

#define AT32_OTGFS_DIEPTSIZ_OFFSET(n)  (0x910 + ((n) << 5)) /* n: 0 ~ 7 */

#define AT32_OTGFS_DIEPTSIZ0_OFFSET    0x0910 /* Device IN endpoint 0 transfer size register */
#define AT32_OTGFS_DIEPTSIZ1_OFFSET    0x0930 /* Device IN endpoint 1 transfer size register */
#define AT32_OTGFS_DIEPTSIZ2_OFFSET    0x0950 /* Device IN endpoint 2 transfer size register */
#define AT32_OTGFS_DIEPTSIZ3_OFFSET    0x0970 /* Device IN endpoint 3 transfer size register */
#define AT32_OTGFS_DIEPTSIZ4_OFFSET    0x0990 /* Device IN endpoint 4 transfer size register */
#define AT32_OTGFS_DIEPTSIZ5_OFFSET    0x09b0 /* Device IN endpoint 5 transfer size register */
#define AT32_OTGFS_DIEPTSIZ6_OFFSET    0x09d0 /* Device IN endpoint 6 transfer size register */
#define AT32_OTGFS_DIEPTSIZ7_OFFSET    0x09f0 /* Device IN endpoint 7 transfer size register */

#define AT32_OTGFS_DTXFSTS_OFFSET(n)   (0x0918 + ((n) << 5)) /* n: 0 ~ 7 */

#define AT32_OTGFS_DTXFSTS0_OFFSET     0x0918 /* Device OUT endpoint-0 TxFIFO status register */
#define AT32_OTGFS_DTXFSTS1_OFFSET     0x0938 /* Device OUT endpoint-1 TxFIFO status register */
#define AT32_OTGFS_DTXFSTS2_OFFSET     0x0958 /* Device OUT endpoint-2 TxFIFO status register */
#define AT32_OTGFS_DTXFSTS3_OFFSET     0x0978 /* Device OUT endpoint-3 TxFIFO status register */
#define AT32_OTGFS_DTXFSTS4_OFFSET     0x0998 /* Device OUT endpoint-4 TxFIFO status register */
#define AT32_OTGFS_DTXFSTS5_OFFSET     0x09b8 /* Device OUT endpoint-5 TxFIFO status register */
#define AT32_OTGFS_DTXFSTS6_OFFSET     0x09d8 /* Device OUT endpoint-6 TxFIFO status register */
#define AT32_OTGFS_DTXFSTS7_OFFSET     0x09f8 /* Device OUT endpoint-7 TxFIFO status register */

#define AT32_OTGFS_DOEP_OFFSET(n)      (0x0b00 + ((n) << 5)) /* n: 0 ~ 7 */

#define AT32_OTGFS_DOEPCTL_EPOFFSET    0x0000 /* Device control OUT endpoint 0 control register */
#define AT32_OTGFS_DOEPINT_EPOFFSET    0x0008 /* Device endpoint-x interrupt register */

#define AT32_OTGFS_DOEPCTL_OFFSET(n)   (0x0b00 + ((n) << 5)) /* n: 0 ~ 7 */

#define AT32_OTGFS_DOEPCTL0_OFFSET     0x00b00 /* Device OUT endpoint 0 control register */
#define AT32_OTGFS_DOEPCTL1_OFFSET     0x00b20 /* Device OUT endpoint 1 control register */
#define AT32_OTGFS_DOEPCTL2_OFFSET     0x00b40 /* Device OUT endpoint 2 control register */
#define AT32_OTGFS_DOEPCTL3_OFFSET     0x00b60 /* Device OUT endpoint 3 control register */
#define AT32_OTGFS_DOEPCTL4_OFFSET     0x00b80 /* Device OUT endpoint 4 control register */
#define AT32_OTGFS_DOEPCTL5_OFFSET     0x00ba0 /* Device OUT endpoint 5 control register */
#define AT32_OTGFS_DOEPCTL6_OFFSET     0x00bc0 /* Device OUT endpoint 6 control register */
#define AT32_OTGFS_DOEPCTL7_OFFSET     0x00be0 /* Device OUT endpoint 7 control register */

#define AT32_OTGFS_DOEPINT_OFFSET(n)   (0x0b08 + ((n) << 5)) /* n: 0 ~ 7 */

#define AT32_OTGFS_DOEPINT0_OFFSET     0x00b08 /* Device endpoint-0 interrupt register */
#define AT32_OTGFS_DOEPINT1_OFFSET     0x00b28 /* Device endpoint-1 interrupt register */
#define AT32_OTGFS_DOEPINT2_OFFSET     0x00b48 /* Device endpoint-2 interrupt register */
#define AT32_OTGFS_DOEPINT3_OFFSET     0x00b68 /* Device endpoint-3 interrupt register */
#define AT32_OTGFS_DOEPINT4_OFFSET     0x00b88 /* Device endpoint-4 interrupt register */
#define AT32_OTGFS_DOEPINT5_OFFSET     0x00ba8 /* Device endpoint-5 interrupt register */
#define AT32_OTGFS_DOEPINT6_OFFSET     0x00bc8 /* Device endpoint-6 interrupt register */
#define AT32_OTGFS_DOEPINT7_OFFSET     0x00be8 /* Device endpoint-7 interrupt register */

#define AT32_OTGFS_DOEPTSIZ_OFFSET(n)  (0x0b10 + ((n) << 5)) /* n: 0 ~ 7 */

#define AT32_OTGFS_DOEPTSIZ0_OFFSET    0x00b10 /* Device OUT endpoint-0 transfer size register */
#define AT32_OTGFS_DOEPTSIZ1_OFFSET    0x00b30 /* Device OUT endpoint-1 transfer size register */
#define AT32_OTGFS_DOEPTSIZ2_OFFSET    0x00b50 /* Device OUT endpoint-2 transfer size register */
#define AT32_OTGFS_DOEPTSIZ3_OFFSET    0x00b70 /* Device OUT endpoint-3 transfer size register */
#define AT32_OTGFS_DOEPTSIZ4_OFFSET    0x00b90 /* Device OUT endpoint-4 transfer size register */
#define AT32_OTGFS_DOEPTSIZ5_OFFSET    0x00bb0 /* Device OUT endpoint-5 transfer size register */
#define AT32_OTGFS_DOEPTSIZ6_OFFSET    0x00bd0 /* Device OUT endpoint-6 transfer size register */
#define AT32_OTGFS_DOEPTSIZ7_OFFSET    0x00bf0 /* Device OUT endpoint-7 transfer size register */

/* Power and clock gating registers */

#define AT32_OTGFS_PCGCCTL_OFFSET      0x0e00 /* Power and clock gating control register */

/* Data FIFO (DFIFO) access registers */

#define AT32_OTGFS_DFIFO_DEP_OFFSET(n) (0x1000 + ((n) << 12)) /* dev: n: 0 ~ 7  host: n: 0 ~ 15 */
#define AT32_OTGFS_DFIFO_HCH_OFFSET(n) (0x1000 + ((n) << 12)) /* dev: n: 0 ~ 7  host: n: 0 ~ 15 */

#define AT32_OTGFS_DFIFO_DEP0_OFFSET   0x1000 /* 0x1000-0x1ffc Device IN/OUT Endpoint 0 DFIFO Write/Read Access */
#define AT32_OTGFS_DFIFO_HCH0_OFFSET   0x1000 /* 0x1000-0x1ffc Host OUT/IN Channel 0 DFIFO Read/Write Access */

#define AT32_OTGFS_DFIFO_DEP1_OFFSET   0x2000 /* 0x2000-0x2ffc Device IN/OUT Endpoint 1 DFIFO Write/Read Access */
#define AT32_OTGFS_DFIFO_HCH1_OFFSET   0x2000 /* 0x2000-0x2ffc Host OUT/IN Channel 1 DFIFO Read/Write Access */

#define AT32_OTGFS_DFIFO_DEP2_OFFSET   0x3000 /* 0x3000-0x3ffc Device IN/OUT Endpoint 2 DFIFO Write/Read Access */
#define AT32_OTGFS_DFIFO_HCH2_OFFSET   0x3000 /* 0x3000-0x3ffc Host OUT/IN Channel 2 DFIFO Read/Write Access */

#define AT32_OTGFS_DFIFO_DEP3_OFFSET   0x4000 /* 0x4000-0x4ffc Device IN/OUT Endpoint 3 DFIFO Write/Read Access */
#define AT32_OTGFS_DFIFO_HCH3_OFFSET   0x4000 /* 0x4000-0x4ffc Host OUT/IN Channel 3 DFIFO Read/Write Access */

#define AT32_OTGFS_DFIFO_DEP4_OFFSET   0x5000 /* 0x1000-0x1ffc Device IN/OUT Endpoint 4 DFIFO Write/Read Access */
#define AT32_OTGFS_DFIFO_HCH4_OFFSET   0x5000 /* 0x1000-0x1ffc Host OUT/IN Channel 4 DFIFO Read/Write Access */

#define AT32_OTGFS_DFIFO_DEP5_OFFSET   0x6000 /* 0x2000-0x2ffc Device IN/OUT Endpoint 5 DFIFO Write/Read Access */
#define AT32_OTGFS_DFIFO_HCH5_OFFSET   0x6000 /* 0x2000-0x2ffc Host OUT/IN Channel 5 DFIFO Read/Write Access */

#define AT32_OTGFS_DFIFO_DEP6_OFFSET   0x7000 /* 0x3000-0x3ffc Device IN/OUT Endpoint 6 DFIFO Write/Read Access */
#define AT32_OTGFS_DFIFO_HCH6_OFFSET   0x7000 /* 0x3000-0x3ffc Host OUT/IN Channel 6 DFIFO Read/Write Access */

#define AT32_OTGFS_DFIFO_DEP7_OFFSET   0x8000 /* 0x4000-0x4ffc Device IN/OUT Endpoint 7 DFIFO Write/Read Access */
#define AT32_OTGFS_DFIFO_HCH7_OFFSET   0x8000 /* 0x4000-0x4ffc Host OUT/IN Channel 7 DFIFO Read/Write Access */

/* Register Addresses *******************************************************/

#define AT32_OTGFS_GOTGCTL             (AT32_OTGFS1_BASE+AT32_OTGFS_GOTGCTL_OFFSET)
#define AT32_OTGFS_GOTGINT             (AT32_OTGFS1_BASE+AT32_OTGFS_GOTGINT_OFFSET)
#define AT32_OTGFS_GAHBCFG             (AT32_OTGFS1_BASE+AT32_OTGFS_GAHBCFG_OFFSET)
#define AT32_OTGFS_GUSBCFG             (AT32_OTGFS1_BASE+AT32_OTGFS_GUSBCFG_OFFSET)
#define AT32_OTGFS_GRSTCTL             (AT32_OTGFS1_BASE+AT32_OTGFS_GRSTCTL_OFFSET)
#define AT32_OTGFS_GINTSTS             (AT32_OTGFS1_BASE+AT32_OTGFS_GINTSTS_OFFSET)
#define AT32_OTGFS_GINTMSK             (AT32_OTGFS1_BASE+AT32_OTGFS_GINTMSK_OFFSET)
#define AT32_OTGFS_GRXSTSR             (AT32_OTGFS1_BASE+AT32_OTGFS_GRXSTSR_OFFSET)
#define AT32_OTGFS_GRXSTSP             (AT32_OTGFS1_BASE+AT32_OTGFS_GRXSTSP_OFFSET)
#define AT32_OTGFS_GRXFSIZ             (AT32_OTGFS1_BASE+AT32_OTGFS_GRXFSIZ_OFFSET)
#define AT32_OTGFS_HNPTXFSIZ           (AT32_OTGFS1_BASE+AT32_OTGFS_HNPTXFSIZ_OFFSET)
#define AT32_OTGFS_DIEPTXF0            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPTXF0_OFFSET)
#define AT32_OTGFS_HNPTXSTS            (AT32_OTGFS1_BASE+AT32_OTGFS_HNPTXSTS_OFFSET)
#define AT32_OTGFS_GCCFG               (AT32_OTGFS1_BASE+AT32_OTGFS_GCCFG_OFFSET)
#define AT32_OTGFS_CID                 (AT32_OTGFS1_BASE+AT32_OTGFS_CID_OFFSET)
#define AT32_OTGFS_HPTXFSIZ            (AT32_OTGFS1_BASE+AT32_OTGFS_HPTXFSIZ_OFFSET)

#define AT32_OTGFS_DIEPTXF(n)          (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPTXF_OFFSET(n))
#define AT32_OTGFS_DIEPTXF1            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPTXF1_OFFSET)
#define AT32_OTGFS_DIEPTXF2            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPTXF2_OFFSET)
#define AT32_OTGFS_DIEPTXF3            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPTXF3_OFFSET)
#define AT32_OTGFS_DIEPTXF4            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPTXF4_OFFSET)
#define AT32_OTGFS_DIEPTXF5            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPTXF5_OFFSET)
#define AT32_OTGFS_DIEPTXF6            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPTXF6_OFFSET)
#define AT32_OTGFS_DIEPTXF7            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPTXF7_OFFSET)

/* Host-mode control and status registers */

#define AT32_OTGFS_HCFG                (AT32_OTGFS1_BASE+AT32_OTGFS_HCFG_OFFSET)
#define AT32_OTGFS_HFIR                (AT32_OTGFS1_BASE+AT32_OTGFS_HFIR_OFFSET)
#define AT32_OTGFS_HFNUM               (AT32_OTGFS1_BASE+AT32_OTGFS_HFNUM_OFFSET)
#define AT32_OTGFS_HPTXSTS             (AT32_OTGFS1_BASE+AT32_OTGFS_HPTXSTS_OFFSET)
#define AT32_OTGFS_HAINT               (AT32_OTGFS1_BASE+AT32_OTGFS_HAINT_OFFSET)
#define AT32_OTGFS_HAINTMSK            (AT32_OTGFS1_BASE+AT32_OTGFS_HAINTMSK_OFFSET)
#define AT32_OTGFS_HPRT                (AT32_OTGFS1_BASE+AT32_OTGFS_HPRT_OFFSET)

#define AT32_OTGFS_CHAN(n)             (AT32_OTGFS1_BASE+AT32_OTGFS_CHAN_OFFSET(n))

#define AT32_OTGFS_HCCHAR(n)           (AT32_OTGFS1_BASE+AT32_OTGFS_HCCHAR_OFFSET(n))
#define AT32_OTGFS_HCCHAR0             (AT32_OTGFS1_BASE+AT32_OTGFS_HCCHAR0_OFFSET)
#define AT32_OTGFS_HCCHAR1             (AT32_OTGFS1_BASE+AT32_OTGFS_HCCHAR1_OFFSET)
#define AT32_OTGFS_HCCHAR2             (AT32_OTGFS1_BASE+AT32_OTGFS_HCCHAR2_OFFSET)
#define AT32_OTGFS_HCCHAR3             (AT32_OTGFS1_BASE+AT32_OTGFS_HCCHAR3_OFFSET)
#define AT32_OTGFS_HCCHAR4             (AT32_OTGFS1_BASE+AT32_OTGFS_HCCHAR4_OFFSET)
#define AT32_OTGFS_HCCHAR5             (AT32_OTGFS1_BASE+AT32_OTGFS_HCCHAR5_OFFSET)
#define AT32_OTGFS_HCCHAR6             (AT32_OTGFS1_BASE+AT32_OTGFS_HCCHAR6_OFFSET)
#define AT32_OTGFS_HCCHAR7             (AT32_OTGFS1_BASE+AT32_OTGFS_HCCHAR7_OFFSET)
#define AT32_OTGFS_HCCHAR8             (AT32_OTGFS1_BASE+AT32_OTGFS_HCCHAR8_OFFSET)
#define AT32_OTGFS_HCCHAR9             (AT32_OTGFS1_BASE+AT32_OTGFS_HCCHAR9_OFFSET)
#define AT32_OTGFS_HCCHAR10            (AT32_OTGFS1_BASE+AT32_OTGFS_HCCHAR10_OFFSET)
#define AT32_OTGFS_HCCHAR11            (AT32_OTGFS1_BASE+AT32_OTGFS_HCCHAR11_OFFSET)
#define AT32_OTGFS_HCCHAR12            (AT32_OTGFS1_BASE+AT32_OTGFS_HCCHAR12_OFFSET)
#define AT32_OTGFS_HCCHAR13            (AT32_OTGFS1_BASE+AT32_OTGFS_HCCHAR13_OFFSET)
#define AT32_OTGFS_HCCHAR14            (AT32_OTGFS1_BASE+AT32_OTGFS_HCCHAR14_OFFSET)
#define AT32_OTGFS_HCCHAR15            (AT32_OTGFS1_BASE+AT32_OTGFS_HCCHAR15_OFFSET)

#define AT32_OTGFS_HCINT(n)            (AT32_OTGFS1_BASE+AT32_OTGFS_HCINT_OFFSET(n))
#define AT32_OTGFS_HCINT0              (AT32_OTGFS1_BASE+AT32_OTGFS_HCINT0_OFFSET)
#define AT32_OTGFS_HCINT1              (AT32_OTGFS1_BASE+AT32_OTGFS_HCINT1_OFFSET)
#define AT32_OTGFS_HCINT2              (AT32_OTGFS1_BASE+AT32_OTGFS_HCINT2_OFFSET)
#define AT32_OTGFS_HCINT3              (AT32_OTGFS1_BASE+AT32_OTGFS_HCINT3_OFFSET)
#define AT32_OTGFS_HCINT4              (AT32_OTGFS1_BASE+AT32_OTGFS_HCINT4_OFFSET)
#define AT32_OTGFS_HCINT5              (AT32_OTGFS1_BASE+AT32_OTGFS_HCINT5_OFFSET)
#define AT32_OTGFS_HCINT6              (AT32_OTGFS1_BASE+AT32_OTGFS_HCINT6_OFFSET)
#define AT32_OTGFS_HCINT7              (AT32_OTGFS1_BASE+AT32_OTGFS_HCINT7_OFFSET)
#define AT32_OTGFS_HCINT8              (AT32_OTGFS1_BASE+AT32_OTGFS_HCINT8_OFFSET)
#define AT32_OTGFS_HCINT9              (AT32_OTGFS1_BASE+AT32_OTGFS_HCINT9_OFFSET)
#define AT32_OTGFS_HCINT10             (AT32_OTGFS1_BASE+AT32_OTGFS_HCINT10_OFFSET)
#define AT32_OTGFS_HCINT11             (AT32_OTGFS1_BASE+AT32_OTGFS_HCINT11_OFFSET)
#define AT32_OTGFS_HCINT12             (AT32_OTGFS1_BASE+AT32_OTGFS_HCINT12_OFFSET)
#define AT32_OTGFS_HCINT13             (AT32_OTGFS1_BASE+AT32_OTGFS_HCINT13_OFFSET)
#define AT32_OTGFS_HCINT14             (AT32_OTGFS1_BASE+AT32_OTGFS_HCINT14_OFFSET)
#define AT32_OTGFS_HCINT15             (AT32_OTGFS1_BASE+AT32_OTGFS_HCINT15_OFFSET)

#define AT32_OTGFS_HCINTMSK(n)         (AT32_OTGFS1_BASE+AT32_OTGFS_HCINTMSK_OFFSET(n))
#define AT32_OTGFS_HCINTMSK0           (AT32_OTGFS1_BASE+AT32_OTGFS_HCINTMSK0_OFFSET)
#define AT32_OTGFS_HCINTMSK1           (AT32_OTGFS1_BASE+AT32_OTGFS_HCINTMSK1_OFFSET)
#define AT32_OTGFS_HCINTMSK2           (AT32_OTGFS1_BASE+AT32_OTGFS_HCINTMSK2_OFFSET)
#define AT32_OTGFS_HCINTMSK3           (AT32_OTGFS1_BASE+AT32_OTGFS_HCINTMSK3_OFFSET)
#define AT32_OTGFS_HCINTMSK4           (AT32_OTGFS1_BASE+AT32_OTGFS_HCINTMSK4_OFFSET)
#define AT32_OTGFS_HCINTMSK5           (AT32_OTGFS1_BASE+AT32_OTGFS_HCINTMSK5_OFFSET)
#define AT32_OTGFS_HCINTMSK6           (AT32_OTGFS1_BASE+AT32_OTGFS_HCINTMSK6_OFFSET)
#define AT32_OTGFS_HCINTMSK7           (AT32_OTGFS1_BASE+AT32_OTGFS_HCINTMSK7_OFFSET)
#define AT32_OTGFS_HCINTMSK8           (AT32_OTGFS1_BASE+AT32_OTGFS_HCINTMSK8_OFFSET)
#define AT32_OTGFS_HCINTMSK9           (AT32_OTGFS1_BASE+AT32_OTGFS_HCINTMSK9_OFFSET)
#define AT32_OTGFS_HCINTMSK10          (AT32_OTGFS1_BASE+AT32_OTGFS_HCINTMSK10_OFFSET)
#define AT32_OTGFS_HCINTMSK11          (AT32_OTGFS1_BASE+AT32_OTGFS_HCINTMSK11_OFFSET)
#define AT32_OTGFS_HCINTMSK12          (AT32_OTGFS1_BASE+AT32_OTGFS_HCINTMSK12_OFFSET)
#define AT32_OTGFS_HCINTMSK13          (AT32_OTGFS1_BASE+AT32_OTGFS_HCINTMSK13_OFFSET)
#define AT32_OTGFS_HCINTMSK14          (AT32_OTGFS1_BASE+AT32_OTGFS_HCINTMSK14_OFFSET)
#define AT32_OTGFS_HCINTMSK15          (AT32_OTGFS1_BASE+AT32_OTGFS_HCINTMSK15_OFFSET)

#define AT32_OTGFS_HCTSIZ(n)           (AT32_OTGFS1_BASE+AT32_OTGFS_HCTSIZ_OFFSET(n))
#define AT32_OTGFS_HCTSIZ0             (AT32_OTGFS1_BASE+AT32_OTGFS_HCTSIZ0_OFFSET)
#define AT32_OTGFS_HCTSIZ1             (AT32_OTGFS1_BASE+AT32_OTGFS_HCTSIZ1_OFFSET)
#define AT32_OTGFS_HCTSIZ2             (AT32_OTGFS1_BASE+AT32_OTGFS_HCTSIZ2_OFFSET)
#define AT32_OTGFS_HCTSIZ3             (AT32_OTGFS1_BASE+AT32_OTGFS_HCTSIZ3_OFFSET)
#define AT32_OTGFS_HCTSIZ4             (AT32_OTGFS1_BASE+AT32_OTGFS_HCTSIZ4_OFFSET)
#define AT32_OTGFS_HCTSIZ5             (AT32_OTGFS1_BASE+AT32_OTGFS_HCTSIZ5_OFFSET)
#define AT32_OTGFS_HCTSIZ6             (AT32_OTGFS1_BASE+AT32_OTGFS_HCTSIZ6_OFFSET)
#define AT32_OTGFS_HCTSIZ7             (AT32_OTGFS1_BASE+AT32_OTGFS_HCTSIZ7_OFFSET)
#define AT32_OTGFS_HCTSIZ8             (AT32_OTGFS1_BASE+AT32_OTGFS_HCTSIZ8_OFFSET)
#define AT32_OTGFS_HCTSIZ9             (AT32_OTGFS1_BASE+AT32_OTGFS_HCTSIZ9_OFFSET)
#define AT32_OTGFS_HCTSIZ10            (AT32_OTGFS1_BASE+AT32_OTGFS_HCTSIZ10_OFFSET)
#define AT32_OTGFS_HCTSIZ11            (AT32_OTGFS1_BASE+AT32_OTGFS_HCTSIZ11_OFFSET)
#define AT32_OTGFS_HCTSIZ12            (AT32_OTGFS1_BASE+AT32_OTGFS_HCTSIZ12_OFFSET)
#define AT32_OTGFS_HCTSIZ13            (AT32_OTGFS1_BASE+AT32_OTGFS_HCTSIZ13_OFFSET)
#define AT32_OTGFS_HCTSIZ14            (AT32_OTGFS1_BASE+AT32_OTGFS_HCTSIZ14_OFFSET)
#define AT32_OTGFS_HCTSIZ15            (AT32_OTGFS1_BASE+AT32_OTGFS_HCTSIZ15_OFFSET)

/* Device-mode control and status registers */

#define AT32_OTGFS_DCFG                (AT32_OTGFS1_BASE+AT32_OTGFS_DCFG_OFFSET)
#define AT32_OTGFS_DCTL                (AT32_OTGFS1_BASE+AT32_OTGFS_DCTL_OFFSET)
#define AT32_OTGFS_DSTS                (AT32_OTGFS1_BASE+AT32_OTGFS_DSTS_OFFSET)
#define AT32_OTGFS_DIEPMSK             (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPMSK_OFFSET)
#define AT32_OTGFS_DOEPMSK             (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPMSK_OFFSET)
#define AT32_OTGFS_DAINT               (AT32_OTGFS1_BASE+AT32_OTGFS_DAINT_OFFSET)
#define AT32_OTGFS_DAINTMSK            (AT32_OTGFS1_BASE+AT32_OTGFS_DAINTMSK_OFFSET)
#define AT32_OTGFS_DIEPEMPMSK          (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPEMPMSK_OFFSET)

#define AT32_OTGFS_DIEP(n)             (AT32_OTGFS1_BASE+AT32_OTGFS_DIEP_OFFSET(n))
#define AT32_OTGFS_DIEPCTL(n)          (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPCTL_OFFSET(n))
#define AT32_OTGFS_DIEPCTL0            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPCTL0_OFFSET)
#define AT32_OTGFS_DIEPCTL1            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPCTL1_OFFSET)
#define AT32_OTGFS_DIEPCTL2            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPCTL2_OFFSET)
#define AT32_OTGFS_DIEPCTL3            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPCTL3_OFFSET)
#define AT32_OTGFS_DIEPCTL4            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPCTL4_OFFSET)
#define AT32_OTGFS_DIEPCTL5            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPCTL5_OFFSET)
#define AT32_OTGFS_DIEPCTL6            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPCTL6_OFFSET)
#define AT32_OTGFS_DIEPCTL7            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPCTL7_OFFSET)

#define AT32_OTGFS_DIEPINT(n)          (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPINT_OFFSET(n))
#define AT32_OTGFS_DIEPINT0            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPINT0_OFFSET)
#define AT32_OTGFS_DIEPINT1            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPINT1_OFFSET)
#define AT32_OTGFS_DIEPINT2            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPINT2_OFFSET)
#define AT32_OTGFS_DIEPINT3            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPINT3_OFFSET)
#define AT32_OTGFS_DIEPINT4            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPINT4_OFFSET)
#define AT32_OTGFS_DIEPINT5            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPINT5_OFFSET)
#define AT32_OTGFS_DIEPINT6            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPINT6_OFFSET)
#define AT32_OTGFS_DIEPINT7            (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPINT7_OFFSET)

#define AT32_OTGFS_DIEPTSIZ(n)         (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPTSIZ_OFFSET(n))
#define AT32_OTGFS_DIEPTSIZ0           (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPTSIZ0_OFFSET)
#define AT32_OTGFS_DIEPTSIZ1           (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPTSIZ1_OFFSET)
#define AT32_OTGFS_DIEPTSIZ2           (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPTSIZ2_OFFSET)
#define AT32_OTGFS_DIEPTSIZ3           (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPTSIZ3_OFFSET)
#define AT32_OTGFS_DIEPTSIZ4           (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPTSIZ4_OFFSET)
#define AT32_OTGFS_DIEPTSIZ5           (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPTSIZ5_OFFSET)
#define AT32_OTGFS_DIEPTSIZ6           (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPTSIZ6_OFFSET)
#define AT32_OTGFS_DIEPTSIZ7           (AT32_OTGFS1_BASE+AT32_OTGFS_DIEPTSIZ7_OFFSET)

#define AT32_OTGFS_DTXFSTS(n)          (AT32_OTGFS1_BASE+AT32_OTGFS_DTXFSTS_OFFSET(n))
#define AT32_OTGFS_DTXFSTS0            (AT32_OTGFS1_BASE+AT32_OTGFS_DTXFSTS0_OFFSET)
#define AT32_OTGFS_DTXFSTS1            (AT32_OTGFS1_BASE+AT32_OTGFS_DTXFSTS1_OFFSET)
#define AT32_OTGFS_DTXFSTS2            (AT32_OTGFS1_BASE+AT32_OTGFS_DTXFSTS2_OFFSET)
#define AT32_OTGFS_DTXFSTS3            (AT32_OTGFS1_BASE+AT32_OTGFS_DTXFSTS3_OFFSET)
#define AT32_OTGFS_DTXFSTS4            (AT32_OTGFS1_BASE+AT32_OTGFS_DTXFSTS4_OFFSET)
#define AT32_OTGFS_DTXFSTS5            (AT32_OTGFS1_BASE+AT32_OTGFS_DTXFSTS5_OFFSET)
#define AT32_OTGFS_DTXFSTS6            (AT32_OTGFS1_BASE+AT32_OTGFS_DTXFSTS6_OFFSET)
#define AT32_OTGFS_DTXFSTS7            (AT32_OTGFS1_BASE+AT32_OTGFS_DTXFSTS7_OFFSET)

#define AT32_OTGFS_DOEP(n)             (AT32_OTGFS1_BASE+AT32_OTGFS_DOEP_OFFSET(n))
#define AT32_OTGFS_DOEPCTL(n)          (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPCTL_OFFSET(n))
#define AT32_OTGFS_DOEPCTL0            (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPCTL0_OFFSET)
#define AT32_OTGFS_DOEPCTL1            (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPCTL1_OFFSET)
#define AT32_OTGFS_DOEPCTL2            (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPCTL2_OFFSET)
#define AT32_OTGFS_DOEPCTL3            (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPCTL3_OFFSET)
#define AT32_OTGFS_DOEPCTL4            (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPCTL4_OFFSET)
#define AT32_OTGFS_DOEPCTL5            (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPCTL5_OFFSET)
#define AT32_OTGFS_DOEPCTL6            (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPCTL6_OFFSET)
#define AT32_OTGFS_DOEPCTL7            (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPCTL7_OFFSET)

#define AT32_OTGFS_DOEPINT(n)          (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPINT_OFFSET(n))
#define AT32_OTGFS_DOEPINT0            (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPINT0_OFFSET)
#define AT32_OTGFS_DOEPINT1            (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPINT1_OFFSET)
#define AT32_OTGFS_DOEPINT2            (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPINT2_OFFSET)
#define AT32_OTGFS_DOEPINT3            (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPINT3_OFFSET)
#define AT32_OTGFS_DOEPINT4            (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPINT4_OFFSET)
#define AT32_OTGFS_DOEPINT5            (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPINT5_OFFSET)
#define AT32_OTGFS_DOEPINT6            (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPINT6_OFFSET)
#define AT32_OTGFS_DOEPINT7            (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPINT7_OFFSET)

#define AT32_OTGFS_DOEPTSIZ(n)         (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPTSIZ_OFFSET(n))
#define AT32_OTGFS_DOEPTSIZ0           (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPTSIZ0_OFFSET)
#define AT32_OTGFS_DOEPTSIZ1           (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPTSIZ1_OFFSET)
#define AT32_OTGFS_DOEPTSIZ2           (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPTSIZ2_OFFSET)
#define AT32_OTGFS_DOEPTSIZ3           (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPTSIZ3_OFFSET)
#define AT32_OTGFS_DOEPTSIZ4           (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPTSIZ4_OFFSET)
#define AT32_OTGFS_DOEPTSIZ5           (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPTSIZ5_OFFSET)
#define AT32_OTGFS_DOEPTSIZ6           (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPTSIZ6_OFFSET)
#define AT32_OTGFS_DOEPTSIZ7           (AT32_OTGFS1_BASE+AT32_OTGFS_DOEPTSIZ7_OFFSET)

/* Power and clock gating registers */

#define AT32_OTGFS_PCGCCTL             (AT32_OTGFS1_BASE+AT32_OTGFS_PCGCCTL_OFFSET)

/* Data FIFO (DFIFO) access registers */

#define AT32_OTGFS_DFIFO_DEP(n)        (AT32_OTGFS1_BASE+AT32_OTGFS_DFIFO_DEP_OFFSET(n))
#define AT32_OTGFS_DFIFO_HCH(n)        (AT32_OTGFS1_BASE+AT32_OTGFS_DFIFO_HCH_OFFSET(n))

#define AT32_OTGFS_DFIFO_DEP0          (AT32_OTGFS1_BASE+AT32_OTGFS_DFIFO_DEP0_OFFSET)
#define AT32_OTGFS_DFIFO_HCH0          (AT32_OTGFS1_BASE+AT32_OTGFS_DFIFO_HCH0_OFFSET)

#define AT32_OTGFS_DFIFO_DEP1          (AT32_OTGFS1_BASE+AT32_OTGFS_DFIFO_DEP1_OFFSET)
#define AT32_OTGFS_DFIFO_HCH1          (AT32_OTGFS1_BASE+AT32_OTGFS_DFIFO_HCH1_OFFSET)

#define AT32_OTGFS_DFIFO_DEP2          (AT32_OTGFS1_BASE+AT32_OTGFS_DFIFO_DEP2_OFFSET)
#define AT32_OTGFS_DFIFO_HCH2          (AT32_OTGFS1_BASE+AT32_OTGFS_DFIFO_HCH2_OFFSET)

#define AT32_OTGFS_DFIFO_DEP3          (AT32_OTGFS1_BASE+AT32_OTGFS_DFIFO_DEP3_OFFSET)
#define AT32_OTGFS_DFIFO_HCH3          (AT32_OTGFS1_BASE+AT32_OTGFS_DFIFO_HCH3_OFFSET)

#define AT32_OTGFS_DFIFO_DEP4          (AT32_OTGFS1_BASE+AT32_OTGFS_DFIFO_DEP4_OFFSET)
#define AT32_OTGFS_DFIFO_HCH4          (AT32_OTGFS1_BASE+AT32_OTGFS_DFIFO_HCH4_OFFSET)

#define AT32_OTGFS_DFIFO_DEP5          (AT32_OTGFS1_BASE+AT32_OTGFS_DFIFO_DEP5_OFFSET)
#define AT32_OTGFS_DFIFO_HCH5          (AT32_OTGFS1_BASE+AT32_OTGFS_DFIFO_HCH5_OFFSET)

#define AT32_OTGFS_DFIFO_DEP6          (AT32_OTGFS1_BASE+AT32_OTGFS_DFIFO_DEP6_OFFSET)
#define AT32_OTGFS_DFIFO_HCH6          (AT32_OTGFS1_BASE+AT32_OTGFS_DFIFO_HCH6_OFFSET)

#define AT32_OTGFS_DFIFO_DEP7          (AT32_OTGFS1_BASE+AT32_OTGFS_DFIFO_DEP7_OFFSET)
#define AT32_OTGFS_DFIFO_HCH7          (AT32_OTGFS1_BASE+AT32_OTGFS_DFIFO_HCH7_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Core global control and status registers */

/* Control and status register */

#define OTGFS_GOTGCTL_CIDSTS            (1 << 16) /* Bit 16: Connector ID status */
#define OTGFS_GOTGCTL_CURMOD            (1 << 21) /* Bit 21: Current Mode of Operation */

/* Interrupt register */

#define OTGFS_GOTGINT_SEDET             (1 << 2)  /* Bit 2: Session end detected */

/* AHB configuration register */

#define OTGFS_GAHBCFG_GINTMSK           (1 << 0)  /* Bit 0: Global interrupt mask */
                                                  /* Bits 1-6: Reserved, must be kept at reset value */
#define OTGFS_GAHBCFG_TXFELVL           (1 << 7)  /* Bit 7: TxFIFO empty level */
#define OTGFS_GAHBCFG_PTXFELVL          (1 << 8)  /* Bit 8: Periodic TxFIFO empty level */
                                                  /* Bits 20-31: Reserved, must be kept at reset value */

/* USB configuration register */

#define OTGFS_GUSBCFG_TOCAL_SHIFT       (0)       /* Bits 0-2: FS timeout calibration */
#define OTGFS_GUSBCFG_TOCAL_MASK        (7 << OTGFS_GUSBCFG_TOCAL_SHIFT)

#define OTGFS_GUSBCFG_TRDT_SHIFT        (10)      /* Bits 10-13: USB turnaround time */
#define OTGFS_GUSBCFG_TRDT_MASK         (15 << OTGFS_GUSBCFG_TRDT_SHIFT)
#  define OTGFS_GUSBCFG_TRDT(n)         ((n) << OTGFS_GUSBCFG_TRDT_SHIFT)

#define OTGFS_GUSBCFG_FHMOD             (1 << 29) /* Bit 29: Force host mode */
#define OTGFS_GUSBCFG_FDMOD             (1 << 30) /* Bit 30: Force device mode */
#define OTGFS_GUSBCFG_CTXPKT            (1 << 31) /* Bit 31: Corrupt Tx packet */

/* Reset register */

#define OTGFS_GRSTCTL_CSRST             (1 << 0)  /* Bit 0: Core soft reset */
#define OTGFS_GRSTCTL_HSRST             (1 << 1)  /* Bit 1: HCLK soft reset */
#define OTGFS_GRSTCTL_FCRST             (1 << 2)  /* Bit 2: Host frame counter reset */
                                                  /* Bit 3 Reserved, must be kept at reset value */
#define OTGFS_GRSTCTL_RXFFLSH           (1 << 4)  /* Bit 4: RxFIFO flush */
#define OTGFS_GRSTCTL_TXFFLSH           (1 << 5)  /* Bit 5: TxFIFO flush */
#define OTGFS_GRSTCTL_TXFNUM_SHIFT      (6)       /* Bits 6-10: TxFIFO number */
#define OTGFS_GRSTCTL_TXFNUM_MASK       (31 << OTGFS_GRSTCTL_TXFNUM_SHIFT)
#  define OTGFS_GRSTCTL_TXFNUM_HNONPER  (0 << OTGFS_GRSTCTL_TXFNUM_SHIFT)   /* Non-periodic TxFIFO flush in host mode */
#  define OTGFS_GRSTCTL_TXFNUM_HPER     (1 << OTGFS_GRSTCTL_TXFNUM_SHIFT)   /* Periodic TxFIFO flush in host mode */
#  define OTGFS_GRSTCTL_TXFNUM_HALL     (16 << OTGFS_GRSTCTL_TXFNUM_SHIFT)  /* Flush all the transmit FIFOs in host mode.*/
#  define OTGFS_GRSTCTL_TXFNUM_D(n)     ((n) << OTGFS_GRSTCTL_TXFNUM_SHIFT) /* TXFIFO n flush in device mode, n=0-15 */
#  define OTGFS_GRSTCTL_TXFNUM_DALL     (16 << OTGFS_GRSTCTL_TXFNUM_SHIFT)  /* Flush all the transmit FIFOs in device mode.*/

/*                                                   Bits 11-31: Reserved,
 *                                                must be kept at reset value
 */
#define OTGFS_GRSTCTL_AHBIDL            (1 << 31) /* Bit 31: AHB master idle */

/* Core interrupt and Interrupt mask registers */

#define OTGFS_GINTSTS_CMOD              (1 << 0)  /* Bit 0: ro Current mode of operation */
#  define OTGFS_GINTSTS_DEVMODE         (0)
#  define OTGFS_GINTSTS_HOSTMODE        (OTGFS_GINTSTS_CMOD)
#define OTGFS_GINT_MMIS                 (1 << 1)  /* Bit 1:  rc_w1 Mode mismatch interrupt */
#define OTGFS_GINT_OTG                  (1 << 2)  /* Bit 2:  ro    OTG interrupt */
#define OTGFS_GINT_SOF                  (1 << 3)  /* Bit 3:  rc_w1 Start of frame */
#define OTGFS_GINT_RXFLVL               (1 << 4)  /* Bit 4:  ro    RxFIFO non-empty */
#define OTGFS_GINT_NPTXFE               (1 << 5)  /* Bit 5:  ro    Non-periodic TxFIFO empty */
#define OTGFS_GINT_GINAKEFF             (1 << 6)  /* Bit 6:  ro    Global IN non-periodic NAK effective */
#define OTGFS_GINT_GONAKEFF             (1 << 7)  /* Bit 7:  Global OUT NAK effective */
#define OTGFS_GINT_RES89                (3 << 8)  /* Bits 8-9:     Reserved, must be kept at reset value */
#define OTGFS_GINT_ESUSP                (1 << 10) /* Bit 10: rc_w1 Early suspend */
#define OTGFS_GINT_USBSUSP              (1 << 11) /* Bit 11: rc_w1 USB suspend */
#define OTGFS_GINT_USBRST               (1 << 12) /* Bit 12: rc_w1 USB reset */
#define OTGFS_GINT_ENUMDNE              (1 << 13) /* Bit 13: rc_w1 Enumeration done */
#define OTGFS_GINT_ISOODRP              (1 << 14) /* Bit 14: rc_w1 Isochronous OUT packet dropped interrupt */
#define OTGFS_GINT_EOPF                 (1 << 15) /* Bit 15: rc_w1 End of periodic frame interrupt */
#define OTGFS_GINT_RES16                (1 << 16) /* Bit 16        Reserved, must be kept at reset value */
#define OTGFS_GINTMSK_EPMISM            (1 << 17) /* Bit 17:       Reserved in GINT rw Endpoint mismatch interrupt mask */
#define OTGFS_GINT_IEP                  (1 << 18) /* Bit 18: ro    IN endpoint interrupt */
#define OTGFS_GINT_OEP                  (1 << 19) /* Bit 19: ro    OUT endpoint interrupt */
#define OTGFS_GINT_IISOIXFR             (1 << 20) /* Bit 20: rc_w1 Incomplete isochronous IN transfer */
#define OTGFS_GINT_IISOOXFR             (1 << 21) /* Bit 21: rc_w1 Incomplete isochronous OUT transfer (device) */
#define OTGFS_GINT_IPXFR                (1 << 21) /* Bit 21:       Incomplete periodic transfer (host) */

#define OTGFS_GINT_RES2223              (3 << 22) /* Bits 22-23: Reserved, must be kept at reset value */

#define OTGFS_GINT_HPRT                 (1 << 24) /* Bit 24: ro    Host port interrupt */
#define OTGFS_GINT_HC                   (1 << 25) /* Bit 25: ro    Host channels interrupt */
#define OTGFS_GINT_PTXFE                (1 << 26) /* Bit 26: ro    Periodic TxFIFO empty */

#define OTGFS_GINT_RES27                (1 << 27) /* Bit 27 Reserved, must be kept at reset value */

#define OTGFS_GINT_CIDSCHG              (1 << 28) /* Bit 28: rc_w1 Connector ID status change */
#define OTGFS_GINT_DISC                 (1 << 29) /* Bit 29: rc_w1 Disconnect detected interrupt */

#define OTGFS_GINT_SRQ                  (1 << 30) /* Bit 30: rc_w1 Session request/new session detected interrupt */

#define OTGFS_GINT_WKUP                 (1 << 31) /* Bit 31: rc_w1 Resume/remote wakeup detected interrupt */

/* Receive status debug read/OTG status read and pop registers
 * (host mode)
 */

#define OTGFS_GRXSTSH_CHNUM_SHIFT       (0)       /* Bits 0-3: Channel number */
#define OTGFS_GRXSTSH_CHNUM_MASK        (15 << OTGFS_GRXSTSH_CHNUM_SHIFT)
#define OTGFS_GRXSTSH_BCNT_SHIFT        (4)       /* Bits 4-14: Byte count */
#define OTGFS_GRXSTSH_BCNT_MASK         (0x7ff << OTGFS_GRXSTSH_BCNT_SHIFT)
#define OTGFS_GRXSTSH_DPID_SHIFT        (15)      /* Bits 15-16: Data PID */
#define OTGFS_GRXSTSH_DPID_MASK         (3 << OTGFS_GRXSTSH_DPID_SHIFT)
#  define OTGFS_GRXSTSH_DPID_DATA0      (0 << OTGFS_GRXSTSH_DPID_SHIFT)
#  define OTGFS_GRXSTSH_DPID_DATA2      (1 << OTGFS_GRXSTSH_DPID_SHIFT)
#  define OTGFS_GRXSTSH_DPID_DATA1      (2 << OTGFS_GRXSTSH_DPID_SHIFT)
#  define OTGFS_GRXSTSH_DPID_MDATA      (3 << OTGFS_GRXSTSH_DPID_SHIFT)
#define OTGFS_GRXSTSH_PKTSTS_SHIFT      (17)      /* Bits 17-20: Packet status */
#define OTGFS_GRXSTSH_PKTSTS_MASK       (15 << OTGFS_GRXSTSH_PKTSTS_SHIFT)
#  define OTGFS_GRXSTSH_PKTSTS_INRECVD  (2 << OTGFS_GRXSTSH_PKTSTS_SHIFT) /* IN data packet received */
#  define OTGFS_GRXSTSH_PKTSTS_INDONE   (3 << OTGFS_GRXSTSH_PKTSTS_SHIFT) /* IN transfer completed */
#  define OTGFS_GRXSTSH_PKTSTS_DTOGERR  (5 << OTGFS_GRXSTSH_PKTSTS_SHIFT) /* Data toggle error */
#  define OTGFS_GRXSTSH_PKTSTS_HALTED   (7 << OTGFS_GRXSTSH_PKTSTS_SHIFT) /* Channel halted */

/*                                                   Bits 21-31: Reserved,
 *                                                must be kept at reset value
 */

/* Receive status debug read/OTG status read and pop registers
 * (device mode)
 */

#define OTGFS_GRXSTSD_EPNUM_SHIFT       (0)       /* Bits 0-3: Endpoint number */
#define OTGFS_GRXSTSD_EPNUM_MASK        (15 << OTGFS_GRXSTSD_EPNUM_SHIFT)
#define OTGFS_GRXSTSD_BCNT_SHIFT        (4)       /* Bits 4-14: Byte count */
#define OTGFS_GRXSTSD_BCNT_MASK         (0x7ff << OTGFS_GRXSTSD_BCNT_SHIFT)
#define OTGFS_GRXSTSD_DPID_SHIFT        (15)      /* Bits 15-16: Data PID */
#define OTGFS_GRXSTSD_DPID_MASK         (3 << OTGFS_GRXSTSD_DPID_SHIFT)
#  define OTGFS_GRXSTSD_DPID_DATA0      (0 << OTGFS_GRXSTSD_DPID_SHIFT)
#  define OTGFS_GRXSTSD_DPID_DATA2      (1 << OTGFS_GRXSTSD_DPID_SHIFT)
#  define OTGFS_GRXSTSD_DPID_DATA1      (2 << OTGFS_GRXSTSD_DPID_SHIFT)
#  define OTGFS_GRXSTSD_DPID_MDATA      (3 << OTGFS_GRXSTSD_DPID_SHIFT)
#define OTGFS_GRXSTSD_PKTSTS_SHIFT      (17)      /* Bits 17-20: Packet status */
#define OTGFS_GRXSTSD_PKTSTS_MASK       (15 << OTGFS_GRXSTSD_PKTSTS_SHIFT)
#  define OTGFS_GRXSTSD_PKTSTS_OUTNAK     (1 << OTGFS_GRXSTSD_PKTSTS_SHIFT) /* Global OUT NAK */
#  define OTGFS_GRXSTSD_PKTSTS_OUTRECVD   (2 << OTGFS_GRXSTSD_PKTSTS_SHIFT) /* OUT data packet received */
#  define OTGFS_GRXSTSD_PKTSTS_OUTDONE    (3 << OTGFS_GRXSTSD_PKTSTS_SHIFT) /* OUT transfer completed */
#  define OTGFS_GRXSTSD_PKTSTS_SETUPDONE  (4 << OTGFS_GRXSTSD_PKTSTS_SHIFT) /* SETUP transaction completed */
#  define OTGFS_GRXSTSD_PKTSTS_SETUPRECVD (6 << OTGFS_GRXSTSD_PKTSTS_SHIFT) /* SETUP data packet received */

#define OTGFS_GRXSTSD_FRMNUM_SHIFT      (21)      /* Bits 21-24: Frame number */
#define OTGFS_GRXSTSD_FRMNUM_MASK       (15 << OTGFS_GRXSTSD_FRMNUM_SHIFT)
                                                  /* Bits 25-31: Reserved, must be kept at reset value */

/* Receive FIFO size register */

#define OTGFS_GRXFSIZ_MASK              (0xffff)

/* Host non-periodic transmit FIFO size register */

#define OTGFS_HNPTXFSIZ_NPTXFSA_SHIFT   (0)       /* Bits 0-15: Non-periodic transmit RAM start address */
#define OTGFS_HNPTXFSIZ_NPTXFSA_MASK    (0xffff << OTGFS_HNPTXFSIZ_NPTXFSA_SHIFT)
#define OTGFS_HNPTXFSIZ_NPTXFD_SHIFT    (16)      /* Bits 16-31: Non-periodic TxFIFO depth */
#define OTGFS_HNPTXFSIZ_NPTXFD_MASK     (0xffff << OTGFS_HNPTXFSIZ_NPTXFD_SHIFT)
#  define OTGFS_HNPTXFSIZ_NPTXFD_MIN    (16 << OTGFS_HNPTXFSIZ_NPTXFD_SHIFT)
#  define OTGFS_HNPTXFSIZ_NPTXFD_MAX    (256 << OTGFS_HNPTXFSIZ_NPTXFD_SHIFT)

/* Endpoint 0 Transmit FIFO size */

#define OTGFS_DIEPTXF0_TX0FD_SHIFT      (0)       /* Bits 0-15: Endpoint 0 transmit RAM start address */
#define OTGFS_DIEPTXF0_TX0FD_MASK       (0xffff << OTGFS_DIEPTXF0_TX0FD_SHIFT)
#define OTGFS_DIEPTXF0_TX0FSA_SHIFT     (16)      /* Bits 16-31: Endpoint 0 TxFIFO depth */
#define OTGFS_DIEPTXF0_TX0FSA_MASK      (0xffff << OTGFS_DIEPTXF0_TX0FSA_SHIFT)
#  define OTGFS_DIEPTXF0_TX0FSA_MIN     (16 << OTGFS_DIEPTXF0_TX0FSA_SHIFT)
#  define OTGFS_DIEPTXF0_TX0FSA_MAX     (256 << OTGFS_DIEPTXF0_TX0FSA_SHIFT)

/* Non-periodic transmit FIFO/queue status register */

#define OTGFS_HNPTXSTS_NPTXFSAV_SHIFT   (0)       /* Bits 0-15: Non-periodic TxFIFO space available */
#define OTGFS_HNPTXSTS_NPTXFSAV_MASK    (0xffff << OTGFS_HNPTXSTS_NPTXFSAV_SHIFT)
#  define OTGFS_HNPTXSTS_NPTXFSAV_FULL  (0 << OTGFS_HNPTXSTS_NPTXFSAV_SHIFT)
#define OTGFS_HNPTXSTS_NPTQXSAV_SHIFT   (16)      /* Bits 16-23: Non-periodic transmit request queue space available */
#define OTGFS_HNPTXSTS_NPTQXSAV_MASK    (0xff << OTGFS_HNPTXSTS_NPTQXSAV_SHIFT)
#  define OTGFS_HNPTXSTS_NPTQXSAV_FULL  (0 << OTGFS_HNPTXSTS_NPTQXSAV_SHIFT)
#define OTGFS_HNPTXSTS_NPTXQTOP_SHIFT   (24)      /* Bits 24-30: Top of the non-periodic transmit request queue */
#define OTGFS_HNPTXSTS_NPTXQTOP_MASK    (0x7f << OTGFS_HNPTXSTS_NPTXQTOP_SHIFT)
#  define OTGFS_HNPTXSTS_TERMINATE      (1 << 24) /* Bit 24: Terminate (last entry for selected channel/endpoint) */
#  define OTGFS_HNPTXSTS_TYPE_SHIFT     (25)      /* Bits 25-26: Status */
#  define OTGFS_HNPTXSTS_TYPE_MASK      (3 << OTGFS_HNPTXSTS_TYPE_SHIFT)
#    define OTGFS_HNPTXSTS_TYPE_INOUT   (0 << OTGFS_HNPTXSTS_TYPE_SHIFT) /* IN/OUT token */
#    define OTGFS_HNPTXSTS_TYPE_ZLP     (1 << OTGFS_HNPTXSTS_TYPE_SHIFT) /* Zero-length transmit packet (device IN/host OUT) */
#    define OTGFS_HNPTXSTS_TYPE_HALT    (3 << OTGFS_HNPTXSTS_TYPE_SHIFT) /* Channel halt command */

#  define OTGFS_HNPTXSTS_CHNUM_SHIFT    (27)      /* Bits 27-30: Channel number */
#  define OTGFS_HNPTXSTS_CHNUM_MASK     (15 << OTGFS_HNPTXSTS_CHNUM_SHIFT)
#  define OTGFS_HNPTXSTS_EPNUM_SHIFT    (27)      /* Bits 27-30: Endpoint number */
#  define OTGFS_HNPTXSTS_EPNUM_MASK     (15 << OTGFS_HNPTXSTS_EPNUM_SHIFT)
                                                  /* Bit 31 Reserved, must be kept at reset value */

/* General core configuration register */

/*                                                   Bits 0-15: Reserved,
 *                                                must be kept at reset value
 */
#define OTGFS_GCCFG_PWRDWN              (1 << 16) /* Bit 16: Power down */
#define OTGFS_GCCFG_LP_MODE             (1 << 17) /* Bit 17: Low power mode */

#define OTGFS_GCCFG_VBUSASEN            (1 << 18) /* Bit 18: Enable the VBUS sensing A device */
#define OTGFS_GCCFG_VBUSBSEN            (1 << 19) /* Bit 19: Enable the VBUS sensing B device */

#define OTGFS_GCCFG_SOFOUTEN            (1 << 20) /* Bit 20: SOF output enable */
#define OTGFS_GCCFG_NOVBUSSENS          (1 << 21) /* Bit 21: VBUS sensing disable option */
                                                  /* Bits 22-31: Reserved, must be kept at reset value */

/* Core ID register  (32-bit product ID) */

/* Host periodic transmit FIFO size register */

#define OTGFS_HPTXFSIZ_PTXSA_SHIFT      (0)       /* Bits 0-15: Host periodic TxFIFO start address */
#define OTGFS_HPTXFSIZ_PTXSA_MASK       (0xffff << OTGFS_HPTXFSIZ_PTXSA_SHIFT)
#define OTGFS_HPTXFSIZ_PTXFD_SHIFT      (16)      /* Bits 16-31: Host periodic TxFIFO depth */
#define OTGFS_HPTXFSIZ_PTXFD_MASK       (0xffff << OTGFS_HPTXFSIZ_PTXFD_SHIFT)

/* Device IN endpoint transmit FIFOn size register */

#define OTGFS_DIEPTXF_INEPTXSA_SHIFT    (0)       /* Bits 0-15: IN endpoint FIFOx transmit RAM start address */
#define OTGFS_DIEPTXF_INEPTXSA_MASK     (0xffff << OTGFS_DIEPTXF_INEPTXSA_SHIFT)
#define OTGFS_DIEPTXF_INEPTXFD_SHIFT    (16)       /* Bits 16-31: IN endpoint TxFIFO depth */
#define OTGFS_DIEPTXF_INEPTXFD_MASK     (0xffff << OTGFS_DIEPTXF_INEPTXFD_SHIFT)
#  define OTGFS_DIEPTXF_INEPTXFD_MIN    (16 << OTGFS_DIEPTXF_INEPTXFD_MASK)

/* Host-mode control and status registers */

/* Host configuration register */

#define OTGFS_HCFG_FSLSPCS_SHIFT        (0)       /* Bits 0-1: FS/LS PHY clock select */
#define OTGFS_HCFG_FSLSPCS_MASK         (3 << OTGFS_HCFG_FSLSPCS_SHIFT)
#  define OTGFS_HCFG_FSLSPCS_FS48MHz    (1 << OTGFS_HCFG_FSLSPCS_SHIFT) /* FS host mode, PHY clock is running at 48 MHz */
#  define OTGFS_HCFG_FSLSPCS_LS48MHz    (1 << OTGFS_HCFG_FSLSPCS_SHIFT) /* LS host mode,  Select 48 MHz PHY clock frequency */
#  define OTGFS_HCFG_FSLSPCS_LS6MHz     (2 << OTGFS_HCFG_FSLSPCS_SHIFT) /* LS host mode, Select 6 MHz PHY clock frequency */

#define OTGFS_HCFG_FSLSS                (1 << 2)  /* Bit 2: FS- and LS-only support */
                                                  /* Bits 31:3 Reserved, must be kept at reset value */

/* Host frame interval register */

#define OTGFS_HFIR_MASK                 (0xffff)
#define OTGFS_HFIR_HFIRRLDCTRL          (1 << 16) /* Reload Control */

/* Host frame number/frame time remaining register */

#define OTGFS_HFNUM_FRNUM_SHIFT         (0)       /* Bits 0-15: Frame number */
#define OTGFS_HFNUM_FRNUM_MASK          (0xffff << OTGFS_HFNUM_FRNUM_SHIFT)
#define OTGFS_HFNUM_FTREM_SHIFT         (16)      /* Bits 16-31: Frame time remaining */
#define OTGFS_HFNUM_FTREM_MASK          (0xffff << OTGFS_HFNUM_FTREM_SHIFT)

/* Host periodic transmit FIFO/queue status register */

#define OTGFS_HPTXSTS_PTXFSAVL_SHIFT    (0)       /* Bits 0-15: Periodic transmit data FIFO space available */
#define OTGFS_HPTXSTS_PTXFSAVL_MASK     (0xffff << OTGFS_HPTXSTS_PTXFSAVL_SHIFT)
#  define OTGFS_HPTXSTS_PTXFSAVL_FULL   (0 << OTGFS_HPTXSTS_PTXFSAVL_SHIFT)
#define OTGFS_HPTXSTS_PTXQSAV_SHIFT     (16)      /* Bits 16-23: Periodic transmit request queue space available */
#define OTGFS_HPTXSTS_PTXQSAV_MASK      (0xff << OTGFS_HPTXSTS_PTXQSAV_SHIFT)
#  define OTGFS_HPTXSTS_PTXQSAV_FULL    (0 << OTGFS_HPTXSTS_PTXQSAV_SHIFT)
#define OTGFS_HPTXSTS_PTXQTOP_SHIFT     (24)      /* Bits 24-31: Top of the periodic transmit request queue */
#define OTGFS_HPTXSTS_PTXQTOP_MASK      (0x7f << OTGFS_HPTXSTS_PTXQTOP_SHIFT)
#  define OTGFS_HPTXSTS_TERMINATE       (1 << 24) /* Bit 24: Terminate (last entry for selected channel/endpoint) */
#  define OTGFS_HPTXSTS_TYPE_SHIFT      (25)      /* Bits 25-26: Type */
#  define OTGFS_HPTXSTS_TYPE_MASK       (3 << OTGFS_HPTXSTS_TYPE_SHIFT)
#    define OTGFS_HPTXSTS_TYPE_INOUT    (0 << OTGFS_HPTXSTS_TYPE_SHIFT) /* IN/OUT token */
#    define OTGFS_HPTXSTS_TYPE_ZLP      (1 << OTGFS_HPTXSTS_TYPE_SHIFT) /* Zero-length transmit packet */
#    define OTGFS_HPTXSTS_TYPE_HALT     (3 << OTGFS_HPTXSTS_TYPE_SHIFT) /* Disable channel command */

#  define OTGFS_HPTXSTS_EPNUM_SHIFT     (27)      /* Bits 27-30: Endpoint number */
#  define OTGFS_HPTXSTS_EPNUM_MASK      (15 << OTGFS_HPTXSTS_EPNUM_SHIFT)
#  define OTGFS_HPTXSTS_CHNUM_SHIFT     (27)      /* Bits 27-30: Channel number */
#  define OTGFS_HPTXSTS_CHNUM_MASK      (15 << OTGFS_HPTXSTS_CHNUM_SHIFT)
#  define OTGFS_HPTXSTS_ODD             (1 << 24) /* Bit 31: Send in odd (vs even) frame */

/* Host all channels interrupt and all channels interrupt mask registers */

#define OTGFS_HAINT(n)                  (1 << (n)) /* Bits 15:0 HAINTM: Channel interrupt */

/* Host port control and status register */

#define OTGFS_HPRT_PCSTS                (1 << 0)  /* Bit 0:  Port connect status */
#define OTGFS_HPRT_PCDET                (1 << 1)  /* Bit 1:  Port connect detected */
#define OTGFS_HPRT_PENA                 (1 << 2)  /* Bit 2:  Port enable */
#define OTGFS_HPRT_PENCHNG              (1 << 3)  /* Bit 3:  Port enable/disable change */
#define OTGFS_HPRT_POCA                 (1 << 4)  /* Bit 4:  Port overcurrent active */
#define OTGFS_HPRT_POCCHNG              (1 << 5)  /* Bit 5:  Port overcurrent change */
#define OTGFS_HPRT_PRES                 (1 << 6)  /* Bit 6:  Port resume */
#define OTGFS_HPRT_PSUSP                (1 << 7)  /* Bit 7:  Port suspend */
#define OTGFS_HPRT_PRST                 (1 << 8)  /* Bit 8:  Port reset */
                                                  /* Bit 9:  Reserved, must be kept at reset value */
#define OTGFS_HPRT_PLSTS_SHIFT          (10)      /* Bits 10-11: Port line status */
#define OTGFS_HPRT_PLSTS_MASK           (3 << OTGFS_HPRT_PLSTS_SHIFT)
#  define OTGFS_HPRT_PLSTS_DP           (1 << 10) /* Bit 10: Logic level of OTG_FS_FS_DP */
#  define OTGFS_HPRT_PLSTS_DM           (1 << 11) /* Bit 11: Logic level of OTG_FS_FS_DM */
#define OTGFS_HPRT_PPWR                 (1 << 12) /* Bit 12: Port power */
#define OTGFS_HPRT_PTCTL_SHIFT          (13)      /* Bits 13-16: Port test control */
#define OTGFS_HPRT_PTCTL_MASK           (15 << OTGFS_HPRT_PTCTL_SHIFT)
#  define OTGFS_HPRT_PTCTL_DISABLED     (0 << OTGFS_HPRT_PTCTL_SHIFT) /* Test mode disabled */
#  define OTGFS_HPRT_PTCTL_J            (1 << OTGFS_HPRT_PTCTL_SHIFT) /* Test_J mode */
#  define OTGFS_HPRT_PTCTL_L            (2 << OTGFS_HPRT_PTCTL_SHIFT) /* Test_K mode */
#  define OTGFS_HPRT_PTCTL_SE0_NAK      (3 << OTGFS_HPRT_PTCTL_SHIFT) /* Test_SE0_NAK mode */
#  define OTGFS_HPRT_PTCTL_PACKET       (4 << OTGFS_HPRT_PTCTL_SHIFT) /* Test_Packet mode */
#  define OTGFS_HPRT_PTCTL_FORCE        (5 << OTGFS_HPRT_PTCTL_SHIFT) /* Test_Force_Enable */

#define OTGFS_HPRT_PSPD_SHIFT           (17)      /* Bits 17-18: Port speed */
#define OTGFS_HPRT_PSPD_MASK            (3 << OTGFS_HPRT_PSPD_SHIFT)
#  define OTGFS_HPRT_PSPD_FS            (1 << OTGFS_HPRT_PSPD_SHIFT) /* Full speed */
#  define OTGFS_HPRT_PSPD_LS            (2 << OTGFS_HPRT_PSPD_SHIFT) /* Low speed */

/*                                                Bits 19-31: Reserved,
 *                                                must be kept at reset value
 */

/* Host channel-n characteristics register */

#define OTGFS_HCCHAR_MPSIZ_SHIFT        (0)       /* Bits 0-10: Maximum packet size */
#define OTGFS_HCCHAR_MPSIZ_MASK         (0x7ff << OTGFS_HCCHAR_MPSIZ_SHIFT)
#define OTGFS_HCCHAR_EPNUM_SHIFT        (11)      /* Bits 11-14: Endpoint number */
#define OTGFS_HCCHAR_EPNUM_MASK         (15 << OTGFS_HCCHAR_EPNUM_SHIFT)
#define OTGFS_HCCHAR_EPDIR              (1 << 15) /* Bit 15: Endpoint direction */
#  define OTGFS_HCCHAR_EPDIR_OUT        (0)
#  define OTGFS_HCCHAR_EPDIR_IN         OTGFS_HCCHAR_EPDIR

/*                                                   Bit 16 Reserved,
 *                                               must be kept at reset value
 */
#define OTGFS_HCCHAR_LSDEV              (1 << 17) /* Bit 17: Low-speed device */
#define OTGFS_HCCHAR_EPTYP_SHIFT        (18)      /* Bits 18-19: Endpoint type */
#define OTGFS_HCCHAR_EPTYP_MASK         (3 << OTGFS_HCCHAR_EPTYP_SHIFT)
#  define OTGFS_HCCHAR_EPTYP_CTRL       (0 << OTGFS_HCCHAR_EPTYP_SHIFT) /* Control */
#  define OTGFS_HCCHAR_EPTYP_ISOC       (1 << OTGFS_HCCHAR_EPTYP_SHIFT) /* Isochronous */
#  define OTGFS_HCCHAR_EPTYP_BULK       (2 << OTGFS_HCCHAR_EPTYP_SHIFT) /* Bulk */
#  define OTGFS_HCCHAR_EPTYP_INTR       (3 << OTGFS_HCCHAR_EPTYP_SHIFT) /* Interrupt */

#define OTGFS_HCCHAR_MCNT_SHIFT         (20)      /* Bits 20-21: Multicount */
#define OTGFS_HCCHAR_MCNT_MASK          (3 << OTGFS_HCCHAR_MCNT_SHIFT)
#define OTGFS_HCCHAR_DAD_SHIFT          (22)      /* Bits 22-28: Device address */
#define OTGFS_HCCHAR_DAD_MASK           (0x7f << OTGFS_HCCHAR_DAD_SHIFT)
#define OTGFS_HCCHAR_ODDFRM             (1 << 29) /* Bit 29: Odd frame */
#define OTGFS_HCCHAR_CHDIS              (1 << 30) /* Bit 30: Channel disable */
#define OTGFS_HCCHAR_CHENA              (1 << 31) /* Bit 31: Channel enable */

/* Host channel-n interrupt and Host channel-0 interrupt mask registers */

#define OTGFS_HCINT_XFRC                (1 << 0)  /* Bit 0:  Transfer completed */
#define OTGFS_HCINT_CHH                 (1 << 1)  /* Bit 1:  Channel halted */
                                                  /* Bit 2:  Reserved, must be kept at reset value */
#define OTGFS_HCINT_STALL               (1 << 3)  /* Bit 3:  STALL response received interrupt */
#define OTGFS_HCINT_NAK                 (1 << 4)  /* Bit 4:  NAK response received interrupt */
#define OTGFS_HCINT_ACK                 (1 << 5)  /* Bit 5:  ACK response received/transmitted interrupt */
#define OTGFS_HCINT_NYET                (1 << 6)  /* Bit 6:  Response received interrupt */
#define OTGFS_HCINT_TXERR               (1 << 7)  /* Bit 7:  Transaction error */
#define OTGFS_HCINT_BBERR               (1 << 8)  /* Bit 8:  Babble error */
#define OTGFS_HCINT_FRMOR               (1 << 9)  /* Bit 9:  Frame overrun */
#define OTGFS_HCINT_DTERR               (1 << 10) /* Bit 10: Data toggle error */

/*                                                   Bits 11-31 Reserved,
 *                                               must be kept at reset value
 */

/* Host channel-n interrupt register */

#define OTGFS_HCTSIZ_XFRSIZ_SHIFT       (0)       /* Bits 0-18: Transfer size */
#define OTGFS_HCTSIZ_XFRSIZ_MASK        (0x7ffff << OTGFS_HCTSIZ_XFRSIZ_SHIFT)
#define OTGFS_HCTSIZ_PKTCNT_SHIFT       (19)      /* Bits 19-28: Packet count */
#define OTGFS_HCTSIZ_PKTCNT_MASK        (0x3ff << OTGFS_HCTSIZ_PKTCNT_SHIFT)
#define OTGFS_HCTSIZ_DPID_SHIFT         (29)      /* Bits 29-30: Data PID */
#define OTGFS_HCTSIZ_DPID_MASK          (3 << OTGFS_HCTSIZ_DPID_SHIFT)
#  define OTGFS_HCTSIZ_DPID_DATA0       (0 << OTGFS_HCTSIZ_DPID_SHIFT)
#  define OTGFS_HCTSIZ_DPID_DATA2       (1 << OTGFS_HCTSIZ_DPID_SHIFT)
#  define OTGFS_HCTSIZ_DPID_DATA1       (2 << OTGFS_HCTSIZ_DPID_SHIFT)
#  define OTGFS_HCTSIZ_DPID_MDATA       (3 << OTGFS_HCTSIZ_DPID_SHIFT) /* Non-control */
#  define OTGFS_HCTSIZ_PID_SETUP        (3 << OTGFS_HCTSIZ_DPID_SHIFT) /* Control */

/*                                                   Bit 31 Reserved,
 *                                                must be kept at reset value
 */

/* Device-mode control and status registers */

/* Device configuration register */

#define OTGFS_DCFG_DSPD_SHIFT           (0)       /* Bits 0-1: Device speed */
#define OTGFS_DCFG_DSPD_MASK            (3 << OTGFS_DCFG_DSPD_SHIFT)
#  define OTGFS_DCFG_DSPD_FS            (3 << OTGFS_DCFG_DSPD_SHIFT) /* Full speed */

#define OTGFS_DCFG_NZLSOHSK             (1 << 2)  /* Bit 2:  Non-zero-length status OUT handshake */
                                                  /* Bit 3:  Reserved, must be kept at reset value */
#define OTGFS_DCFG_DAD_SHIFT            (4)       /* Bits 4-10: Device address */
#define OTGFS_DCFG_DAD_MASK             (0x7f << OTGFS_DCFG_DAD_SHIFT)
#define OTGFS_DCFG_PFIVL_SHIFT          (11)      /* Bits 11-12: Periodic frame interval */
#define OTGFS_DCFG_PFIVL_MASK           (3 << OTGFS_DCFG_PFIVL_SHIFT)
#  define OTGFS_DCFG_PFIVL_80PCT        (0 << OTGFS_DCFG_PFIVL_SHIFT) /* 80% of the frame interval */
#  define OTGFS_DCFG_PFIVL_85PCT        (1 << OTGFS_DCFG_PFIVL_SHIFT) /* 85% of the frame interval */
#  define OTGFS_DCFG_PFIVL_90PCT        (2 << OTGFS_DCFG_PFIVL_SHIFT) /* 90% of the frame interval */
#  define OTGFS_DCFG_PFIVL_95PCT        (3 << OTGFS_DCFG_PFIVL_SHIFT) /* 95% of the frame interval */

/*                                                   Bits 13-31 Reserved,
 *                                                must be kept at reset value
 */

/* Device control register */

#define OTGFS_TESTMODE_DISABLED         (0) /* Test mode disabled */
#define OTGFS_TESTMODE_J                (1) /* Test_J mode */
#define OTGFS_TESTMODE_K                (2) /* Test_K mode */
#define OTGFS_TESTMODE_SE0_NAK          (3) /* Test_SE0_NAK mode */
#define OTGFS_TESTMODE_PACKET           (4) /* Test_Packet mode */
#define OTGFS_TESTMODE_FORCE            (5) /* Test_Force_Enable */

#define OTGFS_DCTL_RWUSIG               (1 << 0)  /* Bit 0:  Remote wakeup signaling */
#define OTGFS_DCTL_SDIS                 (1 << 1)  /* Bit 1:  Soft disconnect */
#define OTGFS_DCTL_GINSTS               (1 << 2)  /* Bit 2:  Global IN NAK status */
#define OTGFS_DCTL_GONSTS               (1 << 3)  /* Bit 3:  Global OUT NAK status */
#define OTGFS_DCTL_TCTL_SHIFT           (4)       /* Bits 4-6: Test control */
#define OTGFS_DCTL_TCTL_MASK            (7 << OTGFS_DCTL_TCTL_SHIFT)
#  define OTGFS_DCTL_TCTL_DISABLED      (0 << OTGFS_DCTL_TCTL_SHIFT) /* Test mode disabled */
#  define OTGFS_DCTL_TCTL_J             (1 << OTGFS_DCTL_TCTL_SHIFT) /* Test_J mode */
#  define OTGFS_DCTL_TCTL_K             (2 << OTGFS_DCTL_TCTL_SHIFT) /* Test_K mode */
#  define OTGFS_DCTL_TCTL_SE0_NAK       (3 << OTGFS_DCTL_TCTL_SHIFT) /* Test_SE0_NAK mode */
#  define OTGFS_DCTL_TCTL_PACKET        (4 << OTGFS_DCTL_TCTL_SHIFT) /* Test_Packet mode */
#  define OTGFS_DCTL_TCTL_FORCE         (5 << OTGFS_DCTL_TCTL_SHIFT) /* Test_Force_Enable */

#define OTGFS_DCTL_SGINAK               (1 << 7)  /* Bit 7:  Set global IN NAK */
#define OTGFS_DCTL_CGINAK               (1 << 8)  /* Bit 8:  Clear global IN NAK */
#define OTGFS_DCTL_SGONAK               (1 << 9)  /* Bit 9:  Set global OUT NAK */
#define OTGFS_DCTL_CGONAK               (1 << 10) /* Bit 10: Clear global OUT NAK */
#define OTGFS_DCTL_POPRGDNE             (1 << 11) /* Bit 11: Power-on programming done */
                                                  /* Bits 12-31: Reserved, must be kept at reset value */

/* Device status register */

#define OTGFS_DSTS_SUSPSTS              (1 << 0)  /* Bit 0: Suspend status */
#define OTGFS_DSTS_ENUMSPD_SHIFT        (1)       /* Bits 1-2: Enumerated speed */
#define OTGFS_DSTS_ENUMSPD_MASK         (3 << OTGFS_DSTS_ENUMSPD_SHIFT)
#  define OTGFS_DSTS_ENUMSPD_FS         (3 << OTGFS_DSTS_ENUMSPD_MASK) /* Full speed */

/*                                                   Bits 4-7: Reserved,
 *                                                must be kept at reset value
 */
#define OTGFS_DSTS_EERR                 (1 << 3)  /* Bit 3: Erratic error */
#define OTGFS_DSTS_SOFFN_SHIFT          (8)       /* Bits 8-21: Frame number of the received SOF */
#define OTGFS_DSTS_SOFFN_MASK           (0x3fff << OTGFS_DSTS_SOFFN_SHIFT)
#define OTGFS_DSTS_SOFFN0               (1 << 8)  /* Bits 8: Frame number even/odd bit */
#define OTGFS_DSTS_SOFFN_EVEN           0
#define OTGFS_DSTS_SOFFN_ODD            OTGFS_DSTS_SOFFN0
                                                  /* Bits 22-31: Reserved, must be kept at reset value */

/* Device IN endpoint common interrupt mask register */

#define OTGFS_DIEPMSK_XFRCM             (1 << 0)  /* Bit 0: Transfer completed interrupt mask */
#define OTGFS_DIEPMSK_EPDM              (1 << 1)  /* Bit 1: Endpoint disabled interrupt mask */
                                                  /* Bit 2:  Reserved, must be kept at reset value */
#define OTGFS_DIEPMSK_TOM               (1 << 3)  /* Bit 3: Timeout condition mask (Non-isochronous endpoints) */
#define OTGFS_DIEPMSK_ITTXFEMSK         (1 << 4)  /* Bit 4: IN token received when TxFIFO empty mask */
#define OTGFS_DIEPMSK_INEPNMM           (1 << 5)  /* Bit 5: IN token received with EP mismatch mask */
#define OTGFS_DIEPMSK_INEPNEM           (1 << 6)  /* Bit 6: IN endpoint NAK effective mask */
#define OTGFS_DIEPMSK_TXFIFOUDRMSK      (1 << 8)  /* Bit 8: FIFO underrun mask */
#define OTGFS_DIEPMSK_BNAINMSK          (1 << 9)  /* Bit 9: BNA interrupt mask */
                                                  /* Bits 10-31: Reserved, must be kept at reset value */

/* Device OUT endpoint common interrupt mask register */

#define OTGFS_DOEPMSK_XFRCM             (1 << 0)  /* Bit 0: Transfer completed interrupt mask */
#define OTGFS_DOEPMSK_EPDM              (1 << 1)  /* Bit 1: Endpoint disabled interrupt mask */
                                                  /* Bit 2:  Reserved, must be kept at reset value */
#define OTGFS_DOEPMSK_STUPM             (1 << 3)  /* Bit 3: SETUP phase done mask */
#define OTGFS_DOEPMSK_OTEPDM            (1 << 4)  /* Bit 4: OUT token received when endpoint disabled mask */
                                                  /* Bits 5-31: Reserved, must be kept at reset value */
#define OTGFS_DOEPMSK_B2BSETUPMSK       (1 << 6)  /* Bit6: Back-to-back SETUP packets received mask */
#define OTGFS_DOEPMSK_OUTPERRMSK        (1 << 8)  /* Bit8: OUTPERRMSK */
#define OTGFS_DOEPMSK_BNAOUTMSK         (1 << 9)  /* Bit9: BNA interrupt mask */

/* Device all endpoints interrupt and All endpoints interrupt mask
 * registers
 */

#define OTGFS_DAINT_IEP_SHIFT           (0)       /* Bits 0-7: IN endpoint interrupt bits */
#define OTGFS_DAINT_IEP_MASK            (0xff << OTGFS_DAINT_IEP_SHIFT)
#  define OTGFS_DAINT_IEP(n)            (1 << (n))
#define OTGFS_DAINT_OEP_SHIFT           (16)      /* Bits 16-23: OUT endpoint interrupt bits */
#define OTGFS_DAINT_OEP_MASK            (0xff << OTGFS_DAINT_OEP_SHIFT)
#  define OTGFS_DAINT_OEP(n)            (1 << ((n)+16))

/* Device IN endpoint FIFO empty interrupt mask register */

#define OTGFS_DIEPEMPMSK(n)             (1 << (n)) /* n: 0 ~ 7 */

/* Device control IN endpoint 0 control register */

#define OTGFS_DIEPCTL0_MPSIZ_SHIFT      (0)       /* Bits 0-1: Maximum packet size */
#define OTGFS_DIEPCTL0_MPSIZ_MASK       (3 << OTGFS_DIEPCTL0_MPSIZ_SHIFT)
#  define OTGFS_DIEPCTL0_MPSIZ_64       (0 << OTGFS_DIEPCTL0_MPSIZ_SHIFT) /* 64 bytes */
#  define OTGFS_DIEPCTL0_MPSIZ_32       (1 << OTGFS_DIEPCTL0_MPSIZ_SHIFT) /* 32 bytes */
#  define OTGFS_DIEPCTL0_MPSIZ_16       (2 << OTGFS_DIEPCTL0_MPSIZ_SHIFT) /* 16 bytes */
#  define OTGFS_DIEPCTL0_MPSIZ_8        (3 << OTGFS_DIEPCTL0_MPSIZ_SHIFT) /* 8 bytes */

/*                                                   Bits 2-14: Reserved,
 *                                                must be kept at reset value
 */
#define OTGFS_DIEPCTL0_USBAEP           (1 << 15) /* Bit 15: USB active endpoint */
                                                  /* Bit 16: Reserved, must be kept at reset value */
#define OTGFS_DIEPCTL0_NAKSTS           (1 << 17) /* Bit 17: NAK status */
#define OTGFS_DIEPCTL0_EPTYP_SHIFT      (18)      /* Bits 18-19: Endpoint type */
#define OTGFS_DIEPCTL0_EPTYP_MASK       (3 << OTGFS_DIEPCTL0_EPTYP_SHIFT)
#  define OTGFS_DIEPCTL0_EPTYP_CTRL     (0 << OTGFS_DIEPCTL0_EPTYP_SHIFT) /* Control (hard-coded) */

/*                                                   Bit 20: Reserved,
 *                                                must be kept at reset value
 */
#define OTGFS_DIEPCTL0_STALL            (1 << 21) /* Bit 21: STALL handshake */
#define OTGFS_DIEPCTL0_TXFNUM_SHIFT     (22)      /* Bits 22-25: TxFIFO number */
#define OTGFS_DIEPCTL0_TXFNUM_MASK      (15 << OTGFS_DIEPCTL0_TXFNUM_SHIFT)
#define OTGFS_DIEPCTL0_CNAK             (1 << 26) /* Bit 26: Clear NAK */
#define OTGFS_DIEPCTL0_SNAK             (1 << 27) /* Bit 27: Set NAK */
                                                  /* Bits 28-29: Reserved, must be kept at reset value */
#define OTGFS_DIEPCTL0_EPDIS            (1 << 30) /* Bit 30: Endpoint disable */
#define OTGFS_DIEPCTL0_EPENA            (1 << 31) /* Bit 31: Endpoint enable */

/* Device control IN endpoint n control register */

#define OTGFS_DIEPCTL_MPSIZ_SHIFT       (0)       /* Bits 0-10: Maximum packet size */
#define OTGFS_DIEPCTL_MPSIZ_MASK        (0x7ff << OTGFS_DIEPCTL_MPSIZ_SHIFT)
                                                  /* Bits 11-14: Reserved, must be kept at reset value */
#define OTGFS_DIEPCTL_USBAEP            (1 << 15) /* Bit 15: USB active endpoint */
#define OTGFS_DIEPCTL_EONUM             (1 << 16) /* Bit 16: Even/odd frame */
#  define OTGFS_DIEPCTL_EVEN            (0)
#  define OTGFS_DIEPCTL_ODD             OTGFS_DIEPCTL_EONUM
#  define OTGFS_DIEPCTL_DATA0           (0)
#  define OTGFS_DIEPCTL_DATA1           OTGFS_DIEPCTL_EONUM
#define OTGFS_DIEPCTL_NAKSTS            (1 << 17) /* Bit 17: NAK status */
#define OTGFS_DIEPCTL_EPTYP_SHIFT       (18)      /* Bits 18-19: Endpoint type */
#define OTGFS_DIEPCTL_EPTYP_MASK        (3 << OTGFS_DIEPCTL_EPTYP_SHIFT)
#  define OTGFS_DIEPCTL_EPTYP_CTRL      (0 << OTGFS_DIEPCTL_EPTYP_SHIFT) /* Control */
#  define OTGFS_DIEPCTL_EPTYP_ISOC      (1 << OTGFS_DIEPCTL_EPTYP_SHIFT) /* Isochronous */
#  define OTGFS_DIEPCTL_EPTYP_BULK      (2 << OTGFS_DIEPCTL_EPTYP_SHIFT) /* Bulk */
#  define OTGFS_DIEPCTL_EPTYP_INTR      (3 << OTGFS_DIEPCTL_EPTYP_SHIFT) /* Interrupt */

/*                                                   Bit 20: Reserved,
 *                                                must be kept at reset value
 */
#define OTGFS_DIEPCTL_STALL             (1 << 21) /* Bit 21: STALL handshake */
#define OTGFS_DIEPCTL_TXFNUM_SHIFT      (22)      /* Bits 22-25: TxFIFO number */
#define OTGFS_DIEPCTL_TXFNUM_MASK       (15 << OTGFS_DIEPCTL_TXFNUM_SHIFT)
#define OTGFS_DIEPCTL_CNAK              (1 << 26) /* Bit 26: Clear NAK */
#define OTGFS_DIEPCTL_SNAK              (1 << 27) /* Bit 27: Set NAK */
#define OTGFS_DIEPCTL_SD0PID            (1 << 28) /* Bit 28: Set DATA0 PID (interrupt/bulk) */
#define OTGFS_DIEPCTL_SEVNFRM           (1 << 28) /* Bit 28: Set even frame (isochronous)) */
#define OTGFS_DIEPCTL_SODDFRM           (1 << 29) /* Bit 29: Set odd frame (isochronous) */
#define OTGFS_DIEPCTL_EPDIS             (1 << 30) /* Bit 30: Endpoint disable */
#define OTGFS_DIEPCTL_EPENA             (1 << 31) /* Bit 31: Endpoint enable */

/* Device endpoint-n interrupt register */

#define OTGFS_DIEPINT_XFRC              (1 << 0)  /* Bit 0:  Transfer completed interrupt */
#define OTGFS_DIEPINT_EPDISD            (1 << 1)  /* Bit 1:  Endpoint disabled interrupt */
                                                  /* Bit 2:  Reserved, must be kept at reset value */
#define OTGFS_DIEPINT_TOC               (1 << 3)  /* Bit 3:  Timeout condition */
#define OTGFS_DIEPINT_ITTXFE            (1 << 4)  /* Bit 4:  IN token received when TxFIFO is empty */
                                                  /* Bit 5:  Reserved, must be kept at reset value */
#define OTGFS_DIEPINT_INEPNE            (1 << 6)  /* Bit 6:  IN endpoint NAK effective */
#define OTGFS_DIEPINT_TXFE              (1 << 7)  /* Bit 7:  Transmit FIFO empty */
                                                  /* Bits 8-31: Reserved, must be kept at reset value */

/* Device IN endpoint 0 transfer size register */

#define OTGFS_DIEPTSIZ0_XFRSIZ_SHIFT    (0)       /* Bits 0-6: Transfer size */
#define OTGFS_DIEPTSIZ0_XFRSIZ_MASK     (0x7f << OTGFS_DIEPTSIZ0_XFRSIZ_SHIFT)
                                                  /* Bits 7-18: Reserved, must be kept at reset value */
#define OTGFS_DIEPTSIZ0_PKTCNT_SHIFT    (19)      /* Bits 19-20: Packet count */
#define OTGFS_DIEPTSIZ0_PKTCNT_MASK     (3 << OTGFS_DIEPTSIZ0_PKTCNT_SHIFT)
                                                  /* Bits 21-31: Reserved, must be kept at reset value */

/* Device IN endpoint n transfer size register */

#define OTGFS_DIEPTSIZ_XFRSIZ_SHIFT     (0)       /* Bits 0-18: Transfer size */
#define OTGFS_DIEPTSIZ_XFRSIZ_MASK      (0x7ffff << OTGFS_DIEPTSIZ_XFRSIZ_SHIFT)
#define OTGFS_DIEPTSIZ_PKTCNT_SHIFT     (19)      /* Bit 19-28: Packet count */
#define OTGFS_DIEPTSIZ_PKTCNT_MASK      (0x3ff << OTGFS_DIEPTSIZ_PKTCNT_SHIFT)
#define OTGFS_DIEPTSIZ_MCNT_SHIFT       (29)      /* Bits 29-30: Multi count */
#define OTGFS_DIEPTSIZ_MCNT_MASK        (3 << OTGFS_DIEPTSIZ_MCNT_SHIFT)
                                                  /* Bit 31: Reserved, must be kept at reset value */

/* Device OUT endpoint TxFIFO status register */

#define OTGFS_DTXFSTS_MASK              (0xffff)

/* Device OUT endpoint 0 control register */

#define OTGFS_DOEPCTL0_MPSIZ_SHIFT      (0)       /* Bits 0-1: Maximum packet size */
#define OTGFS_DOEPCTL0_MPSIZ_MASK       (3 << OTGFS_DOEPCTL0_MPSIZ_SHIFT)
#  define OTGFS_DOEPCTL0_MPSIZ_64       (0 << OTGFS_DOEPCTL0_MPSIZ_SHIFT) /* 64 bytes */
#  define OTGFS_DOEPCTL0_MPSIZ_32       (1 << OTGFS_DOEPCTL0_MPSIZ_SHIFT) /* 32 bytes */
#  define OTGFS_DOEPCTL0_MPSIZ_16       (2 << OTGFS_DOEPCTL0_MPSIZ_SHIFT) /* 16 bytes */
#  define OTGFS_DOEPCTL0_MPSIZ_8        (3 << OTGFS_DOEPCTL0_MPSIZ_SHIFT) /* 8 bytes */

/*                                                   Bits 2-14: Reserved,
 *                                                must be kept at reset value
 */
#define OTGFS_DOEPCTL0_USBAEP           (1 << 15) /* Bit 15: USB active endpoint */
                                                  /* Bit 16: Reserved, must be kept at reset value */
#define OTGFS_DOEPCTL0_NAKSTS           (1 << 17) /* Bit 17: NAK status */
#define OTGFS_DOEPCTL0_EPTYP_SHIFT      (18)      /* Bits 18-19: Endpoint type */
#define OTGFS_DOEPCTL0_EPTYP_MASK       (3 << OTGFS_DOEPCTL0_EPTYP_SHIFT)
#  define OTGFS_DOEPCTL0_EPTYP_CTRL     (0 << OTGFS_DOEPCTL0_EPTYP_SHIFT) /* Control (hard-coded) */

#define OTGFS_DOEPCTL0_SNPM             (1 << 20) /* Bit 20: Snoop mode */
#define OTGFS_DOEPCTL0_STALL            (1 << 21) /* Bit 21: STALL handshake */
                                                  /* Bits 22-25: Reserved, must be kept at reset value */
#define OTGFS_DOEPCTL0_CNAK             (1 << 26) /* Bit 26: Clear NAK */
#define OTGFS_DOEPCTL0_SNAK             (1 << 27) /* Bit 27: Set NAK */
                                                  /* Bits 28-29: Reserved, must be kept at reset value */
#define OTGFS_DOEPCTL0_EPDIS            (1 << 30) /* Bit 30: Endpoint disable */
#define OTGFS_DOEPCTL0_EPENA            (1 << 31) /* Bit 31: Endpoint enable */

/* Device OUT endpoint n control register */

#define OTGFS_DOEPCTL_MPSIZ_SHIFT       (0)       /* Bits 0-10: Maximum packet size */
#define OTGFS_DOEPCTL_MPSIZ_MASK        (0x7ff << OTGFS_DOEPCTL_MPSIZ_SHIFT)
                                                  /* Bits 11-14: Reserved, must be kept at reset value */
#define OTGFS_DOEPCTL_USBAEP            (1 << 15) /* Bit 15: USB active endpoint */
#define OTGFS_DOEPCTL_DPID              (1 << 16) /* Bit 16: Endpoint data PID (interrupt/bulk) */
#  define OTGFS_DOEPCTL_DATA0           (0)
#  define OTGFS_DOEPCTL_DATA1           OTGFS_DOEPCTL_DPID
#define OTGFS_DOEPCTL_EONUM             (1 << 16) /* Bit 16: Even/odd frame (isochronous) */
#  define OTGFS_DOEPCTL_EVEN            (0)
#  define OTGFS_DOEPCTL_ODD             OTGFS_DOEPCTL_EONUM
#define OTGFS_DOEPCTL_NAKSTS            (1 << 17) /* Bit 17: NAK status */
#define OTGFS_DOEPCTL_EPTYP_SHIFT       (18)      /* Bits 18-19: Endpoint type */
#define OTGFS_DOEPCTL_EPTYP_MASK        (3 << OTGFS_DOEPCTL_EPTYP_SHIFT)
#  define OTGFS_DOEPCTL_EPTYP_CTRL      (0 << OTGFS_DOEPCTL_EPTYP_SHIFT) /* Control */
#  define OTGFS_DOEPCTL_EPTYP_ISOC      (1 << OTGFS_DOEPCTL_EPTYP_SHIFT) /* Isochronous */
#  define OTGFS_DOEPCTL_EPTYP_BULK      (2 << OTGFS_DOEPCTL_EPTYP_SHIFT) /* Bulk */
#  define OTGFS_DOEPCTL_EPTYP_INTR      (3 << OTGFS_DOEPCTL_EPTYP_SHIFT) /* Interrupt */

#define OTGFS_DOEPCTL_SNPM              (1 << 20) /* Bit 20: Snoop mode */
#define OTGFS_DOEPCTL_STALL             (1 << 21) /* Bit 21: STALL handshake */
                                                  /* Bits 22-25: Reserved, must be kept at reset value */
#define OTGFS_DOEPCTL_CNAK              (1 << 26) /* Bit 26: Clear NAK */
#define OTGFS_DOEPCTL_SNAK              (1 << 27) /* Bit 27: Set NAK */
#define OTGFS_DOEPCTL_SD0PID            (1 << 28) /* Bit 28: Set DATA0 PID (interrupt/bulk) */
#define OTGFS_DOEPCTL_SEVNFRM           (1 << 28) /* Bit 28: Set even frame (isochronous) */
#define OTGFS_DOEPCTL_SD1PID            (1 << 29) /* Bit 29: Set DATA1 PID (interrupt/bulk) */
#define OTGFS_DOEPCTL_SODDFRM           (1 << 29) /* Bit 29: Set odd frame (isochronous */
#define OTGFS_DOEPCTL_EPDIS             (1 << 30) /* Bit 30: Endpoint disable */
#define OTGFS_DOEPCTL_EPENA             (1 << 31) /* Bit 31: Endpoint enable */

/* Device endpoint-n interrupt register */

#define OTGFS_DOEPINT_XFRC              (1 << 0)  /* Bit 0: Transfer completed interrupt */
#define OTGFS_DOEPINT_EPDISD            (1 << 1)  /* Bit 1: Endpoint disabled interrupt */
                                                  /* Bit 2: Reserved, must be kept at reset value */
#define OTGFS_DOEPINT_SETUP             (1 << 3)  /* Bit 3: SETUP phase done */
#define OTGFS_DOEPINT_OTEPDIS           (1 << 4)  /* Bit 4: OUT token received when endpoint disabled */
                                                  /* Bit 5: Reserved, must be kept at reset value */
#define OTGFS_DOEPINT_B2BSTUP           (1 << 6)  /* Bit 6: Back-to-back SETUP packets received */
                                                  /* Bits 7-31: Reserved, must be kept at reset value */

/* Device OUT endpoint-0 transfer size register */

#define OTGFS_DOEPTSIZ0_XFRSIZ_SHIFT    (0)       /* Bits 0-6: Transfer size */
#define OTGFS_DOEPTSIZ0_XFRSIZ_MASK     (0x7f << OTGFS_DOEPTSIZ0_XFRSIZ_SHIFT)
                                                  /* Bits 7-18: Reserved, must be kept at reset value */
#define OTGFS_DOEPTSIZ0_PKTCNT          (1 << 19) /* Bit 19 PKTCNT: Packet count */
                                                  /* Bits 20-28: Reserved, must be kept at reset value */
#define OTGFS_DOEPTSIZ0_STUPCNT_SHIFT   (29)      /* Bits 29-30: SETUP packet count */
#define OTGFS_DOEPTSIZ0_STUPCNT_MASK    (3 << OTGFS_DOEPTSIZ0_STUPCNT_SHIFT)
                                                  /* Bit 31: Reserved, must be kept at reset value */

/* Device OUT endpoint-n transfer size register */

#define OTGFS_DOEPTSIZ_XFRSIZ_SHIFT     (0)       /* Bits 0-18: Transfer size */
#define OTGFS_DOEPTSIZ_XFRSIZ_MASK      (0x7ffff << OTGFS_DOEPTSIZ_XFRSIZ_SHIFT)
#define OTGFS_DOEPTSIZ_PKTCNT_SHIFT     (19)      /* Bit 19-28: Packet count */
#define OTGFS_DOEPTSIZ_PKTCNT_MASK      (0x3ff << OTGFS_DOEPTSIZ_PKTCNT_SHIFT)
#define OTGFS_DOEPTSIZ_STUPCNT_SHIFT    (29)      /* Bits 29-30: SETUP packet count */
#define OTGFS_DOEPTSIZ_STUPCNT_MASK     (3 << OTGFS_DOEPTSIZ_STUPCNT_SHIFT)
#define OTGFS_DOEPTSIZ_RXDPID_SHIFT     (29)      /* Bits 29-30: Received data PID */
#define OTGFS_DOEPTSIZ_RXDPID_MASK      (3 << OTGFS_DOEPTSIZ_RXDPID_SHIFT)
#  define OTGFS_DOEPTSIZ_RXDPID_DATA0   (0 << OTGFS_DOEPTSIZ_RXDPID_SHIFT)
#  define OTGFS_DOEPTSIZ_RXDPID_DATA2   (1 << OTGFS_DOEPTSIZ_RXDPID_SHIFT)
#  define OTGFS_DOEPTSIZ_RXDPID_DATA1   (2 << OTGFS_DOEPTSIZ_RXDPID_SHIFT)
#  define OTGFS_DOEPTSIZ_RXDPID_MDATA   (3 << OTGFS_DOEPTSIZ_RXDPID_SHIFT)
                                                  /* Bit 31: Reserved, must be kept at reset value */

/* Power and clock gating control register */

#define OTGFS_PCGCCTL_STPPCLK           (1 << 0)  /* Bit 0: Stop PHY clock */

#define OTGFS_PCGCCTL_PHYSUSP           (1 << 4)  /* Bit 4: PHY Suspended */
                                                  /* Bits 5-31: Reserved, must be kept at reset value */

#endif /* __ARCH_ARM_SRC_AT32_HARDWARE_AT32FXXXXX_OTGFS_H */

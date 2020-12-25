/****************************************************************************
 * boards/arm/lc823450/lc823450-xgevk/src/lc823450-xgevk_mux.h
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

#ifndef __BOARDS_ARM_LC823450_LC823450_XGEVK_SRC_LC823450_XGEVK_MUX_H
#define __BOARDS_ARM_LC823450_LC823450_XGEVK_SRC_LC823450_XGEVK_MUX_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * PORT0
 ****************************************************************************/

#define PORT0_MUX                                           \
  0 <<  0 | /* 0: GPIO00, 1:TCLKA0,  2:        3:BCK1 */    \
  0 <<  2 | /* 0: GPIO01, 1:TCLKB0,  2:        3:LRCK1 */   \
  3 <<  4 | /* 0: GPIO02, 1:TIOCB00, 2:DMDIN0, 3:DIN1 */    \
  3 <<  6 | /* 0: GPIO03, 1:TIOCB01, 2:DMCKO0, 3:QSCS */    \
  1 <<  8 | /* 0: GPIO04, 1:TXD1,    2:SDAT20 */            \
  1 << 10 | /* 0: GPIO05, 1:RXD1,    2:SDAT21 */            \
  0 << 12 | /* 0: GPIO06, 1:NCS0 */                         \
  1 << 14 | /* 0: GPIO07, 1:SCL0 */                         \
  1 << 16 | /* 0: GPIO08, 1:SDA0 */                         \
  0 << 18 | /* 0: GPIO09, 1:TIOCA00, 2:SDCLK2, 3:PHI0 */    \
  3 << 20 | /* 0: GPIO0A, 1:TIOCA00, 2:SDCMD2, 3:PHI1 */    \
  0 << 22 | /* 0: GPIO0B, 1:TXD2,    2:TIOCA10 */           \
  0 << 24 | /* 0: GPIO0C, 1:RXD2,    2:TIOCA11 */           \
  1 << 26 | /* 0: GPIO0D, 1:SCK1 */                         \
  1 << 28 | /* 0: GPIO0E, 1:SDI1 */                         \
  1 << 30   /* 0: GPIO0F, 1:SDO1 */

#define PORT0_PUPD                                          \
  0 <<  0 | /* GPIO00  0:non, 1:PU, 2:PD */                 \
  0 <<  2 | /* GPIO01  0:non, 1:PU, 2:PD */                 \
  0 <<  4 | /* GPIO02  0:non, 1:PU, 2:PD */                 \
  1 <<  6 | /* GPIO03  0:non, 1:PU, 2:PD */                 \
  0 <<  8 | /* GPIO04  0:non, 1:PU, 2:PD */                 \
  0 << 10 | /* GPIO05  0:non, 1:PU, 2:PD */                 \
  0 << 12 | /* GPIO06  0:non, 1:PU */                       \
  0 << 14 | /* GPIO07  0:non, 1:PU, 2:PD */                 \
  0 << 16 | /* GPIO08  0:non, 1:PU, 2:PD */                 \
  0 << 18 | /* GPIO09  0:non, 1:PU, 2:PD */                 \
  0 << 20 | /* GPIO0A  0:non, 1:PU, 2:PD */                 \
  0 << 22 | /* GPIO0B  0:non, 1:PU, 2:PD */                 \
  0 << 24 | /* GPIO0C  0:non, 1:PU, 2:PD */                 \
  0 << 26 | /* GPIO0D  0:non, 1:PU, 2:PD */                 \
  0 << 28 | /* GPIO0E  0:non, 1:PU, 2:PD */                 \
  0 << 30   /* GPIO0F  0:non, 1:PU, 2:PD */

#define PORT0_DRV                                           \
  0 <<  0 | /* GPIO00  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 <<  2 | /* GPIO01  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 <<  4 | /* GPIO02  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 <<  6 | /* GPIO03  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 <<  8 | /* GPIO04  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  0 << 10 | /* GPIO05  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  0 << 12 | /* GPIO06  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 14 | /* GPIO07  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 << 16 | /* GPIO08  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 << 18 | /* GPIO09  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  0 << 20 | /* GPIO0A  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  0 << 22 | /* GPIO0B  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 << 24 | /* GPIO0C  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 << 26 | /* GPIO0D  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  0 << 28 | /* GPIO0E  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  0 << 30   /* GPIO0F  0:6mA, 1:---, 2:8mA, 3:10mA */

#define PORT0_DIR                                           \
  1 <<  0 | /* GPIO00  0:in, 1:out     */                   \
  1 <<  1 | /* GPIO01  0:in, 1:out     */                   \
  0 <<  2 | /* GPIO02  0:in, 1:out     */                   \
  0 <<  3 | /* GPIO03  0:in, 1:out     */                   \
  0 <<  4 | /* GPIO04  0:in, 1:out     */                   \
  0 <<  5 | /* GPIO05  0:in, 1:out     */                   \
  1 <<  6 | /* GPIO06  0:in, 1:out     */                   \
  0 <<  7 | /* GPIO07  0:in, 1:out     */                   \
  0 <<  8 | /* GPIO08  0:in, 1:out     */                   \
  1 <<  9 | /* GPIO09  0:in, 1:out     */                   \
  0 << 10 | /* GPIO0A  0:in, 1:out     */                   \
  1 << 11 | /* GPIO0B  0:in, 1:out     */                   \
  1 << 12 | /* GPIO0C  0:in, 1:out     */                   \
  0 << 13 | /* GPIO0D  0:in, 1:out     */                   \
  0 << 14 | /* GPIO0E  0:in, 1:out     */                   \
  0 << 15   /* GPIO0F  0:in, 1:out     */

#define PORT0_DAT                                           \
  0 <<  0 | /* GPIO00  0:0,  1:1       */                   \
  1 <<  1 | /* GPIO01  0:0,  1:1       */                   \
  0 <<  2 | /* GPIO02  0:0,  1:1       */                   \
  0 <<  3 | /* GPIO03  0:0,  1:1       */                   \
  0 <<  4 | /* GPIO04  0:0,  1:1       */                   \
  0 <<  5 | /* GPIO05  0:0,  1:1       */                   \
  0 <<  6 | /* GPIO06  0:0,  1:1       */                   \
  0 <<  7 | /* GPIO07  0:0,  1:1       */                   \
  0 <<  8 | /* GPIO08  0:0,  1:1       */                   \
  0 <<  9 | /* GPIO09  0:0,  1:1       */                   \
  0 << 10 | /* GPIO0A  0:0,  1:1       */                   \
  0 << 11 | /* GPIO0B  0:0,  1:1       */                   \
  0 << 12 | /* GPIO0C  0:0,  1:1       */                   \
  0 << 13 | /* GPIO0D  0:0,  1:1       */                   \
  0 << 14 | /* GPIO0E  0:0,  1:1       */                   \
  0 << 15   /* GPIO0F  0:0,  1:1       */                   \

/****************************************************************************
 * PORT1
 ****************************************************************************/

#define PORT1_MUX                                           \
  3 <<  0 | /* 0: GPIO10, 1:NCS1    2:         3:RXD0 */    \
  1 <<  2 | /* 0: GPIO11, 1:SWP1 */                         \
  1 <<  4 | /* 0: GPIO12, 1:SHOLD1 */                       \
  1 <<  6 | /* 0: GPIO13, 1:BCK1 */                         \
  1 <<  8 | /* 0: GPIO14, 1:LRCK1 */                        \
  1 << 10 | /* 0: GPIO15, 1:DOUT1 */                        \
  0 << 12 | /* 0: GPIO16, 1:NLBEXA0 */                      \
  0 << 14 | /* 0: GPIO17, 1:NRD */                          \
  1 << 16 | /* 0: GPIO18, 1:MCLK0,   2:MCLK1 */             \
  1 << 18 | /* 0: GPIO19, 1:BCK0,    2:DMCKO1 */            \
  1 << 20 | /* 0: GPIO1A, 1:LRCK0,   2:DMDIN1 */            \
  2 << 22 | /* 0: GPIO1B, 1:DIN0,    2:DMDIN0 */            \
  1 << 24 | /* 0: GPIO1C, 1:DOUT0,   2:DMCKO0 */            \
  1 << 26 | /* 0: GPIO1D, 1:SCK0 */                         \
  0 << 28 | /* 0: GPIO1E, 1:SDI0 */                         \
  1 << 30   /* 0: GPIO1F, 1:SDO0 */

#define PORT1_PUPD                                          \
  0 <<  0 | /* GPIO10  0:non, 1:PU */                       \
  0 <<  2 | /* GPIO11  0:non, 1:PU, 2:PD */                 \
  0 <<  4 | /* GPIO12  0:non, 1:PU, 2:PD */                 \
  0 <<  6 | /* GPIO13  0:non, 1:PU, 2:PD */                 \
  0 <<  8 | /* GPIO14  0:non, 1:PU, 2:PD */                 \
  0 << 10 | /* GPIO15  0:non, 1:PU, 2:PD */                 \
  0 << 12 | /* GPIO16  0:non, 1:    2:PD */                 \
  0 << 14 | /* GPIO17  0:non, 1:    2:PD */                 \
  0 << 16 | /* GPIO18  0:non, 1:PU, 2:PD */                 \
  0 << 18 | /* GPIO19  0:non, 1:PU, 2:PD */                 \
  0 << 20 | /* GPIO1A  0:non, 1:PU, 2:PD */                 \
  0 << 22 | /* GPIO1B  0:non, 1:PU, 2:PD */                 \
  0 << 24 | /* GPIO1C  0:non, 1:PU, 2:PD */                 \
  0 << 26 | /* GPIO1D  0:non, 1:PU, 2:PD */                 \
  0 << 28 | /* GPIO1E  0:non, 1:PU, 2:PD */                 \
  0 << 30   /* GPIO1F  0:non, 1:PU, 2:PD */

#define PORT1_DRV                                           \
  0 <<  0 | /* GPIO10  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 <<  2 | /* GPIO11  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  0 <<  4 | /* GPIO12  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  0 <<  6 | /* GPIO13  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 <<  8 | /* GPIO14  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 << 10 | /* GPIO15  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 << 12 | /* GPIO16  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 14 | /* GPIO17  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  3 << 16 | /* GPIO18  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 << 18 | /* GPIO19  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 << 20 | /* GPIO1A  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 << 22 | /* GPIO1B  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 << 24 | /* GPIO1C  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 << 26 | /* GPIO1D  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 << 28 | /* GPIO1E  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 << 30   /* GPIO1F  0:1mA, 1:---, 2:2mA, 3:4mA  */

#define PORT1_DIR                                           \
  0 <<  0 | /* GPIO10  0:in, 1:out     */                   \
  0 <<  1 | /* GPIO11  0:in, 1:out     */                   \
  0 <<  2 | /* GPIO12  0:in, 1:out     */                   \
  0 <<  3 | /* GPIO13  0:in, 1:out     */                   \
  0 <<  4 | /* GPIO14  0:in, 1:out     */                   \
  0 <<  5 | /* GPIO15  0:in, 1:out     */                   \
  1 <<  6 | /* GPIO16  0:in, 1:out     */                   \
  1 <<  7 | /* GPIO17  0:in, 1:out     */                   \
  0 <<  8 | /* GPIO18  0:in, 1:out     */                   \
  0 <<  9 | /* GPIO19  0:in, 1:out     */                   \
  0 << 10 | /* GPIO1A  0:in, 1:out     */                   \
  0 << 11 | /* GPIO1B  0:in, 1:out     */                   \
  0 << 12 | /* GPIO1C  0:in, 1:out     */                   \
  0 << 13 | /* GPIO1D  0:in, 1:out     */                   \
  1 << 14 | /* GPIO1E  0:in, 1:out     */                   \
  0 << 15   /* GPIO1F  0:in, 1:out     */

#define PORT1_DAT                                           \
  0 <<  0 | /* GPIO10  0:0,  1:1       */                   \
  0 <<  1 | /* GPIO11  0:0,  1:1       */                   \
  0 <<  2 | /* GPIO12  0:0,  1:1       */                   \
  0 <<  3 | /* GPIO13  0:0,  1:1       */                   \
  0 <<  4 | /* GPIO14  0:0,  1:1       */                   \
  0 <<  5 | /* GPIO15  0:0,  1:1       */                   \
  0 <<  6 | /* GPIO16  0:0,  1:1       */                   \
  0 <<  7 | /* GPIO17  0:0,  1:1       */                   \
  0 <<  8 | /* GPIO18  0:0,  1:1       */                   \
  0 <<  9 | /* GPIO19  0:0,  1:1       */                   \
  0 << 10 | /* GPIO1A  0:0,  1:1       */                   \
  0 << 11 | /* GPIO1B  0:0,  1:1       */                   \
  0 << 12 | /* GPIO1C  0:0,  1:1       */                   \
  0 << 13 | /* GPIO1D  0:0,  1:1       */                   \
  0 << 14 | /* GPIO1E  0:0,  1:1       */                   \
  0 << 15   /* GPIO1F  0:0,  1:1       */                   \

/****************************************************************************
 * PORT2
 ****************************************************************************/

#define PORT2_MUX                                           \
  0 <<  0 | /* 0: GPIO20, 1:SDCD1,   2:SWO,   3:TDI */      \
  0 <<  2 | /* 0: GPIO21, 1:SDWP1,   2:INS,   3:TDO */      \
  1 <<  4 | /* 0: GPIO22, 1:SDCLK1,  2:SCLK */              \
  1 <<  6 | /* 0: GPIO23, 1:SDCMD1,  2:BS */                \
  1 <<  8 | /* 0: GPIO24, 1:SDAT10,  2:DATA0 */             \
  1 << 10 | /* 0: GPIO25, 1:SDAT11,  2:DATA1 */             \
  1 << 12 | /* 0: GPIO26, 1:SDAT12,  2:DATA2 */             \
  1 << 14 | /* 0: GPIO27, 1:SDAT13,  2:DATA3 */             \
  0 << 16 | /* 0: GPIO28, 1:         2:SDWP2, 3:TMS */      \
  0 << 18 | /* 0: GPIO29, 1:         2:SDCD2, 3:TCK */      \
  0 << 20 | /* 0: GPIO2A, 1:         2:SDRADDR12 */         \
  1 << 22 | /* 0: GPIO2B, 1:SCL1 */                         \
  1 << 24 | /* 0: GPIO2C, 1:SDA1 */                         \
  1 << 26 | /* 0: GPIO2D, 1:DMCK0,   2:SDRADDR11 */         \
  0 << 28 | /* 0: GPIO2E */                                 \
  0 << 30   /* 0: GPIO2F */

#define PORT2_PUPD                                          \
  0 <<  0 | /* GPIO20  0:non, 1:PU, 2:PD */                 \
  0 <<  2 | /* GPIO21  0:non, 1:PU, 2:PD */                 \
  0 <<  4 | /* GPIO22  0:non, 1:PU, 2:PD */                 \
  1 <<  6 | /* GPIO23  0:non, 1:PU, 2:PD */                 \
  1 <<  8 | /* GPIO24  0:non, 1:PU, 2:PD */                 \
  1 << 10 | /* GPIO25  0:non, 1:PU, 2:PD */                 \
  1 << 12 | /* GPIO26  0:non, 1:PU, 2:PD */                 \
  1 << 14 | /* GPIO27  0:non, 1:PU, 2:PD */                 \
  0 << 16 | /* GPIO28  0:non, 1:PU, 2:PD */                 \
  0 << 18 | /* GPIO29  0:non, 1:PU, 2:PD */                 \
  0 << 20 | /* GPIO2A  0:non, 1:PU, 2:PD */                 \
  0 << 22 | /* GPIO2B  0:non, 1:PU, 2:PD */                 \
  0 << 24 | /* GPIO2C  0:non, 1:PU, 2:PD */                 \
  0 << 26 | /* GPIO2D  0:non, 1:PU, 2:PD */                 \
  0 << 28 | /* GPIO2E  0:non, 1:PU, 2:PD */                 \
  0 << 30   /* GPIO2F  0:non, 1:PU, 2:PD */

#define PORT2_DRV                                           \
  0 <<  0 | /* GPIO20  0:---, 1:---, 2:---, 3:---  */       \
  0 <<  2 | /* GPIO21  0:---, 1:---, 2:---, 3:---  */       \
  0 <<  4 | /* GPIO22  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  0 <<  6 | /* GPIO23  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  0 <<  8 | /* GPIO24  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  0 << 10 | /* GPIO25  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  0 << 12 | /* GPIO26  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  0 << 14 | /* GPIO27  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  0 << 16 | /* GPIO28  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 << 18 | /* GPIO29  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 << 20 | /* GPIO2A  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 22 | /* GPIO2B  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 << 24 | /* GPIO2C  0:1mA, 1:---, 2:2mA, 3:4mA  */       \
  0 << 26 | /* GPIO2D  0:2mA, 1:---, 2:4mA, 3:8mA */        \
  0 << 28 | /* GPIO2E  0:1mA, 1:---, 2:2mA, 3:4mA */        \
  0 << 30   /* GPIO2F  0:1mA, 1:---, 2:2mA, 3:4mA */

#define PORT2_DIR                                           \
  0 <<  0 | /* GPIO20  0:in, 1:out     */                   \
  1 <<  1 | /* GPIO21  0:in, 1:out     */                   \
  0 <<  2 | /* GPIO22  0:in, 1:out     */                   \
  0 <<  3 | /* GPIO23  0:in, 1:out     */                   \
  0 <<  4 | /* GPIO24  0:in, 1:out     */                   \
  0 <<  5 | /* GPIO25  0:in, 1:out     */                   \
  0 <<  6 | /* GPIO26  0:in, 1:out     */                   \
  0 <<  7 | /* GPIO27  0:in, 1:out     */                   \
  0 <<  8 | /* GPIO28  0:in, 1:out     */                   \
  0 <<  9 | /* GPIO29  0:in, 1:out     */                   \
  0 << 10 | /* GPIO2A  0:in, 1:out     */                   \
  0 << 11 | /* GPIO2B  0:in, 1:out     */                   \
  0 << 12 | /* GPIO2C  0:in, 1:out     */                   \
  0 << 13 | /* GPIO2D  0:in, 1:out     */                   \
  0 << 14 | /* GPIO2E  0:in, 1:out     */                   \
  1 << 15   /* GPIO2F  0:in, 1:out     */

#define PORT2_DAT                                           \
  0 <<  0 | /* GPIO20  0:0,  1:1       */                   \
  1 <<  1 | /* GPIO21  0:0,  1:1       */                   \
  0 <<  2 | /* GPIO22  0:0,  1:1       */                   \
  0 <<  3 | /* GPIO23  0:0,  1:1       */                   \
  0 <<  4 | /* GPIO24  0:0,  1:1       */                   \
  0 <<  5 | /* GPIO25  0:0,  1:1       */                   \
  0 <<  6 | /* GPIO26  0:0,  1:1       */                   \
  0 <<  7 | /* GPIO27  0:0,  1:1       */                   \
  0 <<  8 | /* GPIO28  0:0,  1:1       */                   \
  0 <<  9 | /* GPIO29  0:0,  1:1       */                   \
  0 << 10 | /* GPIO2A  0:0,  1:1       */                   \
  0 << 11 | /* GPIO2B  0:0,  1:1       */                   \
  0 << 12 | /* GPIO2C  0:0,  1:1       */                   \
  0 << 13 | /* GPIO2D  0:0,  1:1       */                   \
  0 << 14 | /* GPIO2E  0:0,  1:1       */                   \
  0 << 15   /* GPIO2F  0:0,  1:1       */                   \

/****************************************************************************
 * PORT3
 ****************************************************************************/

#define PORT3_MUX                                           \
  0 <<  0 | /* 0: GPIO30, 1:NWRENWRL */                     \
  3 <<  2 | /* 0: GPIO31, 1:NHBNWRH, 2:     3:TXDO */       \
  0 <<  4 | /* 0: GPIO32, 1:EXA1 */                         \
  0 <<  6 | /* 0: GPIO33, 1:EXA2 */                         \
  0 <<  8 | /* 0: GPIO34, 1:EXA3 */                         \
  0 << 10 | /* 0: GPIO35, 1:EXA4 */                         \
  0 << 12 | /* 0: GPIO36, 1:EXA5 */                         \
  0 << 14 | /* 0: GPIO37, 1:EXA6 */                         \
  0 << 16 | /* 0: GPIO38, 1:EXA7 */                         \
  0 << 18 | /* 0: GPIO39, 1:EXA8 */                         \
  0 << 20 | /* 0: GPIO3A, 1:EXA9 */                         \
  0 << 22 | /* 0: GPIO3B, 1:EXA10 */                        \
  0 << 24 | /* 0: GPIO3C, 1:EXA11 */                        \
  0 << 26 | /* 0: GPIO3D, 1:EXA12 */                        \
  0 << 28 | /* 0: GPIO3E, 1:EXA13 */                        \
  0 << 30   /* 0: GPIO3F, 1:EXA14 */

#define PORT3_PUPD                                          \
  0 <<  0 | /* GPIO30  0:non, 1:    2:PD */                 \
  0 <<  2 | /* GPIO31  0:non, 1:    2:PD */                 \
  0 <<  4 | /* GPIO32  0:non, 1:    2:PD */                 \
  0 <<  6 | /* GPIO33  0:non, 1:    2:PD */                 \
  0 <<  8 | /* GPIO34  0:non, 1:    2:PD */                 \
  0 << 10 | /* GPIO35  0:non, 1:    2:PD */                 \
  0 << 12 | /* GPIO36  0:non, 1:    2:PD */                 \
  0 << 14 | /* GPIO37  0:non, 1:    2:PD */                 \
  0 << 16 | /* GPIO38  0:non, 1:    2:PD */                 \
  0 << 18 | /* GPIO39  0:non, 1:    2:PD */                 \
  0 << 20 | /* GPIO3A  0:non, 1:    2:PD */                 \
  0 << 22 | /* GPIO3B  0:non, 1:    2:PD */                 \
  0 << 24 | /* GPIO3C  0:non, 1:    2:PD */                 \
  0 << 26 | /* GPIO3D  0:non, 1:    2:PD */                 \
  0 << 28 | /* GPIO3E  0:non, 1:    2:PD */                 \
  0 << 30   /* GPIO3F  0:non, 1:    2:PD */

#define PORT3_DRV                                           \
  0 <<  0 | /* GPIO30  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 <<  2 | /* GPIO31  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 <<  4 | /* GPIO32  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 <<  6 | /* GPIO33  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 <<  8 | /* GPIO34  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 10 | /* GPIO35  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 12 | /* GPIO36  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 14 | /* GPIO37  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 16 | /* GPIO38  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 18 | /* GPIO39  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 20 | /* GPIO3A  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 22 | /* GPIO3B  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 24 | /* GPIO3C  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 26 | /* GPIO3D  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 28 | /* GPIO3E  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 30   /* GPIO3F  0:2mA, 1:---, 2:4mA, 3:8mA  */

#define PORT3_DIR                                           \
  1 <<  0 | /* GPIO30  0:in, 1:out     */                   \
  0 <<  1 | /* GPIO31  0:in, 1:out     */                   \
  0 <<  2 | /* GPIO32  0:in, 1:out     */                   \
  0 <<  3 | /* GPIO33  0:in, 1:out     */                   \
  0 <<  4 | /* GPIO34  0:in, 1:out     */                   \
  0 <<  5 | /* GPIO35  0:in, 1:out     */                   \
  0 <<  6 | /* GPIO36  0:in, 1:out     */                   \
  0 <<  7 | /* GPIO37  0:in, 1:out     */                   \
  0 <<  8 | /* GPIO38  0:in, 1:out     */                   \
  0 <<  9 | /* GPIO39  0:in, 1:out     */                   \
  0 << 10 | /* GPIO3A  0:in, 1:out     */                   \
  0 << 11 | /* GPIO3B  0:in, 1:out     */                   \
  0 << 12 | /* GPIO3C  0:in, 1:out     */                   \
  0 << 13 | /* GPIO3D  0:in, 1:out     */                   \
  0 << 14 | /* GPIO3E  0:in, 1:out     */                   \
  0 << 15   /* GPIO3F  0:in, 1:out     */

#define PORT3_DAT                                           \
  0 <<  0 | /* GPIO30  0:0,  1:1       */                   \
  0 <<  1 | /* GPIO31  0:0,  1:1       */                   \
  0 <<  2 | /* GPIO32  0:0,  1:1       */                   \
  0 <<  3 | /* GPIO33  0:0,  1:1       */                   \
  0 <<  4 | /* GPIO34  0:0,  1:1       */                   \
  0 <<  5 | /* GPIO35  0:0,  1:1       */                   \
  0 <<  6 | /* GPIO36  0:0,  1:1       */                   \
  0 <<  7 | /* GPIO37  0:0,  1:1       */                   \
  0 <<  8 | /* GPIO38  0:0,  1:1       */                   \
  0 <<  9 | /* GPIO39  0:0,  1:1       */                   \
  0 << 10 | /* GPIO3A  0:0,  1:1       */                   \
  0 << 11 | /* GPIO3B  0:0,  1:1       */                   \
  0 << 12 | /* GPIO3C  0:0,  1:1       */                   \
  0 << 13 | /* GPIO3D  0:0,  1:1       */                   \
  0 << 14 | /* GPIO3E  0:0,  1:1       */                   \
  0 << 15   /* GPIO3F  0:0,  1:1       */

/****************************************************************************
 * PORT4
 ****************************************************************************/

#define PORT4_MUX                                           \
  0 <<  0 | /* 0: GPIO40, 1:EXA15 */                        \
  0 <<  2 | /* 0: GPIO41, 1:EXA16 */                        \
  0 <<  4 | /* 0: GPIO42, 1:EXA17 */                        \
  0 <<  6 | /* 0: GPIO43, 1:EXA18 */                        \
  0 <<  8 | /* 0: GPIO44, 1:EXA19 */                        \
  0 << 10 | /* 0: GPIO45, 1:EXA20 */                        \
  0 << 12 | /* 0: GPIO46, 1:EXD0 */                         \
  0 << 14 | /* 0: GPIO47, 1:EXD1 */                         \
  0 << 16 | /* 0: GPIO48, 1:EXD2 */                         \
  0 << 18 | /* 0: GPIO49, 1:EXD3 */                         \
  0 << 20 | /* 0: GPIO4A, 1:EXD4 */                         \
  0 << 22 | /* 0: GPIO4B, 1:EXD5 */                         \
  0 << 24 | /* 0: GPIO4C, 1:EXD6 */                         \
  0 << 26 | /* 0: GPIO4D, 1:EXD7 */                         \
  0 << 28 | /* 0: GPIO4E, 1:EXD8 */                         \
  0 << 30   /* 0: GPIO4F, 1:EXD9 */

#define PORT4_PUPD                                          \
  0 <<  0 | /* GPIO40  0:non, 1:    2:PD */                 \
  0 <<  2 | /* GPIO41  0:non, 1:    2:PD */                 \
  0 <<  4 | /* GPIO42  0:non, 1:    2:PD */                 \
  0 <<  6 | /* GPIO43  0:non, 1:    2:PD */                 \
  0 <<  8 | /* GPIO44  0:non, 1:    2:PD */                 \
  0 << 10 | /* GPIO45  0:non, 1:    2:PD */                 \
  0 << 12 | /* GPIO46  0:non, 1:    2:PD */                 \
  0 << 14 | /* GPIO47  0:non, 1:    2:PD */                 \
  0 << 16 | /* GPIO48  0:non, 1:    2:PD */                 \
  0 << 18 | /* GPIO49  0:non, 1:    2:PD */                 \
  0 << 20 | /* GPIO4A  0:non, 1:    2:PD */                 \
  0 << 22 | /* GPIO4B  0:non, 1:    2:PD */                 \
  0 << 24 | /* GPIO4C  0:non, 1:    2:PD */                 \
  0 << 26 | /* GPIO4D  0:non, 1:    2:PD */                 \
  0 << 28 | /* GPIO4E  0:non, 1:    2:PD */                 \
  0 << 30   /* GPIO4F  0:non, 1:    2:PD */

#define PORT4_DRV                                           \
  0 <<  0 | /* GPIO40  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 <<  2 | /* GPIO41  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 <<  4 | /* GPIO42  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 <<  6 | /* GPIO43  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 <<  8 | /* GPIO44  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 10 | /* GPIO45  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 12 | /* GPIO46  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 14 | /* GPIO47  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 16 | /* GPIO48  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 18 | /* GPIO49  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 20 | /* GPIO4A  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 22 | /* GPIO4B  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 24 | /* GPIO4C  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 26 | /* GPIO4D  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 28 | /* GPIO4E  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 30   /* GPIO4F  0:2mA, 1:---, 2:4mA, 3:8mA  */

#define PORT4_DIR                                           \
  0 <<  0 | /* GPIO40  0:in, 1:out     */                   \
  0 <<  1 | /* GPIO41  0:in, 1:out     */                   \
  0 <<  2 | /* GPIO42  0:in, 1:out     */                   \
  0 <<  3 | /* GPIO43  0:in, 1:out     */                   \
  0 <<  4 | /* GPIO44  0:in, 1:out     */                   \
  0 <<  5 | /* GPIO45  0:in, 1:out     */                   \
  0 <<  6 | /* GPIO46  0:in, 1:out     */                   \
  1 <<  7 | /* GPIO47  0:in, 1:out     */                   \
  1 <<  8 | /* GPIO48  0:in, 1:out     */                   \
  1 <<  9 | /* GPIO49  0:in, 1:out     */                   \
  0 << 10 | /* GPIO4A  0:in, 1:out     */                   \
  0 << 11 | /* GPIO4B  0:in, 1:out     */                   \
  0 << 12 | /* GPIO4C  0:in, 1:out     */                   \
  0 << 13 | /* GPIO4D  0:in, 1:out     */                   \
  0 << 14 | /* GPIO4E  0:in, 1:out     */                   \
  0 << 15   /* GPIO4F  0:in, 1:out     */

#define PORT4_DAT                                           \
  0 <<  0 | /* GPIO40  0:0,  1:1       */                   \
  0 <<  1 | /* GPIO41  0:0,  1:1       */                   \
  0 <<  2 | /* GPIO42  0:0,  1:1       */                   \
  0 <<  3 | /* GPIO43  0:0,  1:1       */                   \
  0 <<  4 | /* GPIO44  0:0,  1:1       */                   \
  0 <<  5 | /* GPIO45  0:0,  1:1       */                   \
  0 <<  6 | /* GPIO46  0:0,  1:1       */                   \
  0 <<  7 | /* GPIO47  0:0,  1:1       */                   \
  0 <<  8 | /* GPIO48  0:0,  1:1       */                   \
  0 <<  9 | /* GPIO49  0:0,  1:1       */                   \
  0 << 10 | /* GPIO4A  0:0,  1:1       */                   \
  0 << 11 | /* GPIO4B  0:0,  1:1       */                   \
  0 << 12 | /* GPIO4C  0:0,  1:1       */                   \
  0 << 13 | /* GPIO4D  0:0,  1:1       */                   \
  0 << 14 | /* GPIO4E  0:0,  1:1       */                   \
  0 << 15   /* GPIO4F  0:0,  1:1       */

/****************************************************************************
 * PORT5
 ****************************************************************************/

#define PORT5_MUX                                           \
  0 <<  0 | /* 0: GPIO50, 1:EXD10 */                        \
  0 <<  2 | /* 0: GPIO51, 1:EXD11 */                        \
  0 <<  4 | /* 0: GPIO52, 1:EXD12 */                        \
  0 <<  6 | /* 0: GPIO53, 1:EXD13 */                        \
  0 <<  8 | /* 0: GPIO54, 1:EXD14 */                        \
  0 << 10 | /* 0: GPIO55, 1:EXD25 */                        \
  1 << 12 | /* 0: GPIO56, 1:CTS1,   2:SDAT22 3:RXD0 */      \
  1 << 14 | /* 0: GPIO57, 1:RTS1,   2:SDAT23 3:TXD0 */      \
  3 << 16 | /* 0: GPIO58, 1:DMCKO1, 2:      3:SWDCLK */     \
  3 << 18   /* 0: GPIO59, 1:DMDIN1, 2:      3:SWDIO */

#define PORT5_PUPD                                          \
  0 <<  0 | /* GPIO50  0:non, 1:    2:PD */                 \
  0 <<  2 | /* GPIO51  0:non, 1:    2:PD */                 \
  0 <<  4 | /* GPIO52  0:non, 1:    2:PD */                 \
  0 <<  6 | /* GPIO53  0:non, 1:    2:PD */                 \
  0 <<  8 | /* GPIO54  0:non, 1:    2:PD */                 \
  0 << 10 | /* GPIO55  0:non, 1:    2:PD */                 \
  2 << 12 | /* GPIO56  0:non, 1:PU, 2:PD */                 \
  2 << 14 | /* GPIO57  0:non, 1:PU, 2:PD */                 \
  0 << 16 | /* GPIO58  0:non, 1:PU, 2:PD */                 \
  0 << 18   /* GPIO59  0:non, 1:PU */

#define PORT5_DRV                                           \
  0 <<  0 | /* GPIO50  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 <<  2 | /* GPIO51  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 <<  4 | /* GPIO52  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 <<  6 | /* GPIO53  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 <<  8 | /* GPIO54  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 10 | /* GPIO55  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 12 | /* GPIO56  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 14 | /* GPIO57  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  0 << 16 | /* GPIO58  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  0 << 18   /* GPIO59  0:1mA, 1:---, 2:2mA, 3:4mA  */

#define PORT5_DIR                                           \
  0 <<  0 | /* GPIO50  0:in, 1:out     */                   \
  0 <<  1 | /* GPIO51  0:in, 1:out     */                   \
  0 <<  2 | /* GPIO52  0:in, 1:out     */                   \
  0 <<  3 | /* GPIO53  0:in, 1:out     */                   \
  0 <<  4 | /* GPIO54  0:in, 1:out     */                   \
  0 <<  5 | /* GPIO55  0:in, 1:out     */                   \
  0 <<  6 | /* GPIO56  0:in, 1:out     */                   \
  0 <<  7 | /* GPIO57  0:in, 1:out     */                   \
  0 <<  8 | /* GPIO58  0:in, 1:out     */                   \
  0 <<  9   /* GPIO59  0:in, 1:out     */

#define PORT5_DAT                                           \
  0 <<  0 | /* GPIO50  0:0,  1:1       */                   \
  0 <<  1 | /* GPIO51  0:0,  1:1       */                   \
  0 <<  2 | /* GPIO52  0:0,  1:1       */                   \
  0 <<  3 | /* GPIO53  0:0,  1:1       */                   \
  0 <<  4 | /* GPIO54  0:0,  1:1       */                   \
  0 <<  5 | /* GPIO55  0:0,  1:1       */                   \
  0 <<  6 | /* GPIO56  0:0,  1:1       */                   \
  0 <<  7 | /* GPIO57  0:0,  1:1       */                   \
  0 <<  8 | /* GPIO58  0:0,  1:1       */                   \
  0 <<  9   /* GPIO59  0:0,  1:1       */

/****************************************************************************
 * PORT6
 ****************************************************************************/

#define PORT6_PUPD                                          \
  1 <<  2 | /* SDCMD0      0:non, 1:    2:PD */             \
  1 <<  4 | /* SDAT03-0    0:non, 1:    2:PD */             \
  0 <<  6 | /* SDRDATA15-0 0:non, 1:    2:PD */             \
  0 <<  8 | /* BMODE0      0:non, 1:    2:PD */             \
  0 << 10 | /* BMODE1      0:non, 1:    2:PD */             \
  0 << 12 | /* XTALINFO0   0:non, 1:PU, 2:PD */             \
  0 << 14   /* XTALINFO1   0:non, 1:PU, 2:PD */

#define PORT6_DRV                                           \
  2 <<  0 | /* SDCLK0  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  2 <<  2 | /* SDCMD0  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  2 <<  4 | /* SDAT03  0:6mA, 1:---, 2:8mA, 3:10mA */       \
  0 <<  6 | /* SDRDA1  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 <<  8 | /* SDRAD1  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 10 | /* SDRBA1  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 12 | /* SDRCAS  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 14 | /* SDRCKE  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 16 | /* SDRCLK  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 18 | /* SDRCS   0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 20 | /* SDRDQM  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 22 | /* XTALI0  0:2mA, 1:---, 2:4mA, 3:8mA  */       \
  0 << 24   /* XTALI1  0:2mA, 1:---, 2:4mA, 3:8mA  */

#endif /* __BOARDS_ARM_LC823450_LC823450_XGEVK_SRC_LC823450_XGEVK_MUX_H */

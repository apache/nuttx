/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_port.h
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

#ifndef __ARCH_RENESAS_SRC_RX65N_RX65N_PORT_H
#define __ARCH_RENESAS_SRC_RX65N_RX65N_PORT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Port Direction Register (PDR) */

/* Pmn Direction Control (B7 - B0) */

#define _00_PM0_MODE_NOT_USED   (0x00u) /* PM0 not used */
#define _00_PM0_MODE_INPUT      (0x00u) /* PM0 as input */
#define _01_PM0_MODE_OUTPUT     (0x01u) /* PM0 as output */
#define _00_PM1_MODE_NOT_USED   (0x00u) /* PM1 not used */
#define _00_PM1_MODE_INPUT      (0x00u) /* PM1 as input */
#define _02_PM1_MODE_OUTPUT     (0x02u) /* PM1 as output */
#define _00_PM2_MODE_NOT_USED   (0x00u) /* PM2 not used */
#define _00_PM2_MODE_INPUT      (0x00u) /* PM2 as input */
#define _04_PM2_MODE_OUTPUT     (0x04u) /* PM2 as output */
#define _00_PM3_MODE_NOT_USED   (0x00u) /* PM3 not used */
#define _00_PM3_MODE_INPUT      (0x00u) /* PM3 as input */
#define _08_PM3_MODE_OUTPUT     (0x08u) /* PM3 as output */
#define _00_PM4_MODE_NOT_USED   (0x00u) /* PM4 not used */
#define _00_PM4_MODE_INPUT      (0x00u) /* PM4 as input */
#define _10_PM4_MODE_OUTPUT     (0x10u) /* PM4 as output */
#define _00_PM5_MODE_NOT_USED   (0x00u) /* PM5 not used */
#define _00_PM5_MODE_INPUT      (0x00u) /* PM5 as input */
#define _20_PM5_MODE_OUTPUT     (0x20u) /* PM5 as output */
#define _00_PM6_MODE_NOT_USED   (0x00u) /* PM6 not used */
#define _00_PM6_MODE_INPUT      (0x00u) /* PM6 as input */
#define _40_PM6_MODE_OUTPUT     (0x40u) /* PM6 as output */
#define _00_PM7_MODE_NOT_USED   (0x00u) /* PM7 not used */
#define _00_PM7_MODE_INPUT      (0x00u) /* PM7 as input */
#define _80_PM7_MODE_OUTPUT     (0x80u) /* PM7 as output */
#define _50_PDR0_DEFAULT        (0x50u) /* PDR0 default value */
#define _03_PDR1_DEFAULT        (0x03u) /* PDR1 default value */
#define _80_PDR5_DEFAULT        (0x80u) /* PDR5 default value */
#define _30_PDR8_DEFAULT        (0x30u) /* PDR8 default value */
#define _F0_PDR9_DEFAULT        (0xf0u) /* PDR9 default value */
#define _DF_PDRF_DEFAULT        (0xdfu) /* PDRF default value */
#define _D7_PDRJ_DEFAULT        (0xd7u) /* PDRJ default value */

/* Port Output Data Register (PODR) */

/* Pmn Output Data Store (B7 - B0) */

#define _00_PM0_OUTPUT_0        (0x00u) /* output low at B0 */
#define _01_PM0_OUTPUT_1        (0x01u) /* output high at B0 */
#define _00_PM1_OUTPUT_0        (0x00u) /* output low at B1 */
#define _02_PM1_OUTPUT_1        (0x02u) /* output high at B1 */
#define _00_PM2_OUTPUT_0        (0x00u) /* output low at B2 */
#define _04_PM2_OUTPUT_1        (0x04u) /* output high at B2 */
#define _00_PM3_OUTPUT_0        (0x00u) /* output low at B3 */
#define _08_PM3_OUTPUT_1        (0x08u) /* output high at B3 */
#define _00_PM4_OUTPUT_0        (0x00u) /* output low at B4 */
#define _10_PM4_OUTPUT_1        (0x10u) /* output high at B4 */
#define _00_PM5_OUTPUT_0        (0x00u) /* output low at B5 */
#define _20_PM5_OUTPUT_1        (0x20u) /* output high at B5 */
#define _00_PM6_OUTPUT_0        (0x00u) /* output low at B6 */
#define _40_PM6_OUTPUT_1        (0x40u) /* output high at B6 */
#define _00_PM7_OUTPUT_0        (0x00u) /* output low at B7 */
#define _80_PM7_OUTPUT_1        (0x80u) /* output high at B7 */

/* Open Drain Control Register 0 (ODR0) */

/* Pmn Output Type Select (Pm0 to Pm3)  */

#define _00_PM0_CMOS_OUTPUT     (0x00u) /* CMOS output */
#define _01_PM0_NCH_OPEN_DRAIN  (0x01u) /* NMOS open-drain output */
#define _00_PM1_CMOS_OUTPUT     (0x00u) /* CMOS output */
#define _04_PM1_NCH_OPEN_DRAIN  (0x04u) /* NMOS open-drain output */
#define _08_PM1_PCH_OPEN_DRAIN  (0x08u) /* PMOS open-drain output,for PE1 only */
#define _00_PM2_CMOS_OUTPUT     (0x00u) /* CMOS output */
#define _10_PM2_NCH_OPEN_DRAIN  (0x10u) /* NMOS open-drain output */
#define _00_PM3_CMOS_OUTPUT     (0x00u) /* CMOS output */
#define _40_PM3_NCH_OPEN_DRAIN  (0x40u) /* NMOS open-drain output */

/* Open Drain Control Register 1 (ODR1) */

/* Pmn Output Type Select (Pm4 to Pm7) */

#define _00_PM4_CMOS_OUTPUT     (0x00u) /* CMOS output */
#define _01_PM4_NCH_OPEN_DRAIN  (0x01u) /* NMOS open-drain output */
#define _00_PM5_CMOS_OUTPUT     (0x00u) /* CMOS output */
#define _04_PM5_NCH_OPEN_DRAIN  (0x04u) /* NMOS open-drain output */
#define _00_PM6_CMOS_OUTPUT     (0x00u) /* CMOS output */
#define _10_PM6_NCH_OPEN_DRAIN  (0x10u) /* NMOS open-drain output */
#define _00_PM7_CMOS_OUTPUT     (0x00u) /* CMOS output */
#define _40_PM7_NCH_OPEN_DRAIN  (0x40u) /* NMOS open-drain output */

/* Pull-Up Control Register (PCR) */

/* Pmn Input Pull-Up Resistor Control (B7 - B0) */

/* PM0 pull-up resistor not connected */

#define _00_PM0_PULLUP_OFF      (0x00u)

/* PM0 pull-up resistor connected */

#define _01_PM0_PULLUP_ON       (0x01u)

/* PM1 pull-up resistor not connected */

#define _00_PM1_PULLUP_OFF      (0x00u)

/* PM1 pull-up resistor connected */

#define _02_PM1_PULLUP_ON       (0x02u)

/* PM2 Pull-up resistor not connected */

#define _00_PM2_PULLUP_OFF      (0x00u)

/* PM2 pull-up resistor connected */

#define _04_PM2_PULLUP_ON       (0x04u)

/* PM3 pull-up resistor not connected */

#define _00_PM3_PULLUP_OFF      (0x00u)

/* PM3 pull-up resistor connected */

#define _08_PM3_PULLUP_ON       (0x08u)

/* PM4 pull-up resistor not connected */

#define _00_PM4_PULLUP_OFF      (0x00u)

/* PM4 pull-up resistor connected */

#define _10_PM4_PULLUP_ON       (0x10u)

/* PM5 pull-up resistor not connected */

#define _00_PM5_PULLUP_OFF      (0x00u)

/* PM5 pull-up resistor connected */

#define _20_PM5_PULLUP_ON       (0x20u)

/* PM6 pull-up resistor not connected */

#define _00_PM6_PULLUP_OFF      (0x00u)

/* PM6 pull-up resistor connected */

#define _40_PM6_PULLUP_ON       (0x40u)

/* PM7 pull-up resistor not connected */

#define _00_PM7_PULLUP_OFF      (0x00u)

/* Pm7 pull-up resistor connected */

#define _80_PM7_PULLUP_ON       (0x80u)

/* Drive Capacity Control Register (DSCR) */

/* Pmn Drive Capacity Control (B7 - B0)   */

#define _00_PM0_HIDRV_OFF       (0x00u) /* PM0 Normal drive output */
#define _01_PM0_HIDRV_ON        (0x01u) /* PM0 High-drive output */
#define _00_PM1_HIDRV_OFF       (0x00u) /* PM1 Normal drive output */
#define _02_PM1_HIDRV_ON        (0x02u) /* PM1 High-drive output */
#define _00_PM2_HIDRV_OFF       (0x00u) /* PM2 Normal drive output */
#define _04_PM2_HIDRV_ON        (0x04u) /* PM2 High-drive output */
#define _00_PM3_HIDRV_OFF       (0x00u) /* PM3 Normal drive output */
#define _08_PM3_HIDRV_ON        (0x08u) /* PM3 High-drive output */
#define _00_PM4_HIDRV_OFF       (0x00u) /* PM4 Normal drive output */
#define _10_PM4_HIDRV_ON        (0x10u) /* PM4 High-drive output */
#define _00_PM5_HIDRV_OFF       (0x00u) /* PM5 Normal drive output */
#define _20_PM5_HIDRV_ON        (0x20u) /* PM5 High-drive output */
#define _00_PM6_HIDRV_OFF       (0x00u) /* PM6 Normal drive output */
#define _40_PM6_HIDRV_ON        (0x40u) /* PM6 High-drive output */
#define _00_PM7_HIDRV_OFF       (0x00u) /* PM7 Normal drive output */
#define _80_PM7_HIDRV_ON        (0x80u) /* PM7 High-drive output */

/* Drive Capacity Control Register 2 (DSCR2) */

/* Pmn Drive Capacity Control (B7 - B0) */

/* PM0 Normal drive/high-drive output */

#define _00_PM0_HISPEED_OFF     (0x00u)

/* PM0 High-speed interface high-drive output */

#define _01_PM0_HISPEED_ON      (0x01u)

/* PM1 Normal drive/high-drive output */

#define _00_PM1_HISPEED_OFF     (0x00u)

/* PM1 High-speed interface high-drive output */

#define _02_PM1_HISPEED_ON      (0x02u)

/* PM2 Normal drive/high-drive output */

#define _00_PM2_HISPEED_OFF     (0x00u)

/* PM2 High-speed interface high-drive output */

#define _04_PM2_HISPEED_ON      (0x04u)

/* PM3 Normal drive/high-drive output */

#define _00_PM3_HISPEED_OFF     (0x00u)

/* PM3 High-speed interface high-drive output */

#define _08_PM3_HISPEED_ON      (0x08u)

/* PM4 Normal drive/high-drive output */

#define _00_PM4_HISPEED_OFF     (0x00u)

/* PM4 High-speed interface high-drive output */

#define _10_PM4_HISPEED_ON      (0x10u)

/* PM5 Normal drive/high-drive output */

#define _00_PM5_HISPEED_OFF     (0x00u)

/* PM5 High-speed interface high-drive output */

#define _20_PM5_HISPEED_ON      (0x20u)

/* PM6 Normal drive/high-drive output */

#define _00_PM6_HISPEED_OFF     (0x00u)

/* PM6 High-speed interface high-drive output */

#define _40_PM6_HISPEED_ON      (0x40u)

/* PM7 Normal drive/high-drive output */

#define _00_PM7_HISPEED_OFF     (0x00u)

/* PM7 High-speed interface high-drive output */

#define _80_PM7_HISPEED_ON      (0x80u)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: r_port_create
 *
 * Description:
 *   Initializes Ports of rx65n
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_port_create(void);

/****************************************************************************
 * Name: r_ether_port_configuration
 *
 * Description:
 *   Initializes Ethernet Ports of rx65n
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_EMAC0
void r_ether_port_configuration(void);
#endif

#endif /* __ARCH_RENESAS_SRC_RX65N_RX65N_PORT_H */

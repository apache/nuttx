/****************************************************************************
 * drivers/lcd/pcd8544.h
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

/* Definitions for the PCD8544 LCD Display */

#ifndef __DRIVERS_LCD_PCD8544_H
#define __DRIVERS_LCD_PCD8544_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PCD8544_MAX_BANKS           6
#define PCD8544_MAX_COLS            84

/* Fundamental Commands *****************************************************/

#define PCD8544_NOP                 0x00     /* 0x00: No Operation */
#define PCD8544_FUNC_SET            (1 << 5) /* Used to select extend mode */
#define PCD8544_POWER_DOWN          (1 << 2) /* Enable power-down mode */
#define PCD8544_MODEV               (1 << 1) /* Enable Vertical Addressing */
#define PCD8544_MODEH               (1 << 0) /* Enable extended instruction set */

/* Command with Instructon Set H = 0 */

#define PCD8544_DISP_CTRL           (1 << 3) /* sets display configuration */
#define PCD8544_DISP_BLANK          0x00     /* display blank */
#define PCD8544_DISP_ALLON          0x01     /* all display segments on */
#define PCD8544_DISP_NORMAL         0x04     /* normal mode */
#define PCD8544_DISP_INVERT         0x05     /* inverse video mode */

#define PCD8544_SET_Y_ADDR          (1 << 6) /* Set the Y bank 0-5 */
#define PCD8544_SET_X_ADDR          (1 << 7) /* Set the X bank 0-83 */

/* Command with Instructon Set H = 1 */

#define PCD8544_TEMP_COEF           (1 << 2) /* set Temperature Coefficient */
#define PCD8544_BIAS_SYSTEM         (1 << 4) /* set Bias System */
#define PCD8544_BIAS_BS0            (1 << 0) /* set Bias System */
#define PCD8544_BIAS_BS1            (1 << 1) /* set Bias System */
#define PCD8544_BIAS_BS2            (1 << 2) /* set Bias System */
#define PCD8544_WRITE_VOP           (1 << 7) /* write Vop to register*/
#define PCD8544_VOP0                (1 << 0) /* Vop0 */
#define PCD8544_VOP1                (1 << 1) /* Vop1 */
#define PCD8544_VOP2                (1 << 2) /* Vop2 */
#define PCD8544_VOP3                (1 << 3) /* Vop3 */
#define PCD8544_VOP4                (1 << 4) /* Vop4 */
#define PCD8544_VOP5                (1 << 5) /* Vop5 */
#define PCD8544_VOP6                (1 << 6) /* Vop6 */

#endif /* __DRIVERS_LCD_PCD8544_H */

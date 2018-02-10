/**************************************************************************************
 * drivers/lcd/pcd8544.h
 * Definitions for the PCD8544 LCD Display
 *
 *   Copyright (C) 2017 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************************/

#ifndef __DRIVERS_LCD_PCD8544_H
#define __DRIVERS_LCD_PCD8544_H

/**************************************************************************************
 * Included Files
 **************************************************************************************/

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

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

/****************************************************************************
 * include/nuttx/lcd/ili9488.h
 *
 *   Copyright (c) 2011, Atmel Corporation
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   - Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the disclaimer below.
 *
 *   - Atmel's name may not be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_LCD_ILI9488_H
#define __INCLUDE_NUTTX_LCD_ILI9488_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ILI9488 ID code */

#define ILI9488_DEVICE_CODE                         0x9488
#define ILI9488_LCD_WIDTH                           320
#define ILI9488_LCD_HEIGHT                          480
#define ILI9488_SELF_TEST_OK                        0xc0

/* Level 1 Commands (from the display Datasheet) */

#define ILI9488_CMD_NOP                             0x00
#define ILI9488_CMD_SOFTWARE_RESET                  0x01
#define ILI9488_CMD_READ_DISP_ID                    0x04
#define ILI9488_CMD_READ_ERROR_DSI                  0x05
#define ILI9488_CMD_READ_DISP_STATUS                0x09
#define ILI9488_CMD_READ_DISP_POWER_MODE            0x0a
#define ILI9488_CMD_READ_DISP_MADCTRL               0x0b
#define ILI9488_CMD_READ_DISP_PIXEL_FORMAT          0x0c
#define ILI9488_CMD_READ_DISP_IMAGE_MODE            0x0d
#define ILI9488_CMD_READ_DISP_SIGNAL_MODE           0x0e
#define ILI9488_CMD_READ_DISP_SELF_DIAGNOSTIC       0x0f
#define ILI9488_CMD_ENTER_SLEEP_MODE                0x10
#define ILI9488_CMD_SLEEP_OUT                       0x11
#define ILI9488_CMD_PARTIAL_MODE_ON                 0x12
#define ILI9488_CMD_NORMAL_DISP_MODE_ON             0x13
#define ILI9488_CMD_DISP_INVERSION_OFF              0x20
#define ILI9488_CMD_DISP_INVERSION_ON               0x21
#define ILI9488_CMD_PIXEL_OFF                       0x22
#define ILI9488_CMD_PIXEL_ON                        0x23
#define ILI9488_CMD_DISPLAY_OFF                     0x28
#define ILI9488_CMD_DISPLAY_ON                      0x29
#define ILI9488_CMD_COLUMN_ADDRESS_SET              0x2a
#define ILI9488_CMD_PAGE_ADDRESS_SET                0x2b
#define ILI9488_CMD_MEMORY_WRITE                    0x2c
#define ILI9488_CMD_MEMORY_READ                     0x2e
#define ILI9488_CMD_PARTIAL_AREA                    0x30
#define ILI9488_CMD_VERT_SCROLL_DEFINITION          0x33
#define ILI9488_CMD_TEARING_EFFECT_LINE_OFF         0x34
#define ILI9488_CMD_TEARING_EFFECT_LINE_ON          0x35
#define ILI9488_CMD_MEMORY_ACCESS_CONTROL           0x36
#define ILI9488_CMD_VERT_SCROLL_START_ADDRESS       0x37
#define ILI9488_CMD_IDLE_MODE_OFF                   0x38
#define ILI9488_CMD_IDLE_MODE_ON                    0x39
#define ILI9488_CMD_COLMOD_PIXEL_FORMAT_SET         0x3a
#define ILI9488_CMD_WRITE_MEMORY_CONTINUE           0x3c
#define ILI9488_CMD_READ_MEMORY_CONTINUE            0x3e
#define ILI9488_CMD_SET_TEAR_SCANLINE               0x44
#define ILI9488_CMD_GET_SCANLINE                    0x45
#define ILI9488_CMD_WRITE_DISPLAY_BRIGHTNESS        0x51
#define ILI9488_CMD_READ_DISPLAY_BRIGHTNESS         0x52
#define ILI9488_CMD_WRITE_CTRL_DISPLAY              0x53
#define ILI9488_CMD_READ_CTRL_DISPLAY               0x54
#define ILI9488_CMD_WRITE_CONTENT_ADAPT_BRIGHTNESS  0x55
#define ILI9488_CMD_READ_CONTENT_ADAPT_BRIGHTNESS   0x56
#define ILI9488_CMD_WRITE_MIN_CAB_LEVEL             0x5e
#define ILI9488_CMD_READ_MIN_CAB_LEVEL              0x5f
#define ILI9488_CMD_READ_ABC_SELF_DIAG_RES          0x68
#define ILI9488_CMD_READ_ID1                        0xda
#define ILI9488_CMD_READ_ID2                        0xdb
#define ILI9488_CMD_READ_ID3                        0xdc

/* Level 2 Commands (from the display Datasheet) */

#define ILI9488_CMD_INTERFACE_MODE_CONTROL          0xb0
#define ILI9488_CMD_FRAME_RATE_CONTROL_NORMAL       0xb1
#define ILI9488_CMD_FRAME_RATE_CONTROL_IDLE_8COLOR  0xb2
#define ILI9488_CMD_FRAME_RATE_CONTROL_PARTIAL      0xb3
#define ILI9488_CMD_DISPLAY_INVERSION_CONTROL       0xb4
#define ILI9488_CMD_BLANKING_PORCH_CONTROL          0xb5
#define ILI9488_CMD_DISPLAY_FUNCTION_CONTROL        0xb6
#define ILI9488_CMD_ENTRY_MODE_SET                  0xb7
#define ILI9488_CMD_BACKLIGHT_CONTROL_1             0xb9
#define ILI9488_CMD_BACKLIGHT_CONTROL_2             0xba
#define ILI9488_CMD_HS_LANES_CONTROL                0xbe
#define ILI9488_CMD_POWER_CONTROL_1                 0xc0
#define ILI9488_CMD_POWER_CONTROL_2                 0xc1
#define ILI9488_CMD_POWER_CONTROL_NORMAL_3          0xc2
#define ILI9488_CMD_POWER_CONTROL_IDEL_4            0xc3
#define ILI9488_CMD_POWER_CONTROL_PARTIAL_5         0xc4
#define ILI9488_CMD_VCOM_CONTROL_1                  0xc5
#define ILI9488_CMD_CABC_CONTROL_1                  0xc6
#define ILI9488_CMD_CABC_CONTROL_2                  0xc8
#define ILI9488_CMD_CABC_CONTROL_3                  0xc9
#define ILI9488_CMD_CABC_CONTROL_4                  0xca
#define ILI9488_CMD_CABC_CONTROL_5                  0xcb
#define ILI9488_CMD_CABC_CONTROL_6                  0xcc
#define ILI9488_CMD_CABC_CONTROL_7                  0xcd
#define ILI9488_CMD_CABC_CONTROL_8                  0xce
#define ILI9488_CMD_CABC_CONTROL_9                  0xcf
#define ILI9488_CMD_NVMEM_WRITE                     0xd0
#define ILI9488_CMD_NVMEM_PROTECTION_KEY            0xd1
#define ILI9488_CMD_NVMEM_STATUS_READ               0xd2
#define ILI9488_CMD_READ_ID4                        0xd3
#define ILI9488_CMD_ADJUST_CONTROL_1                0xd7
#define ILI9488_CMD_READ_ID_VERSION                 0xd8
#define ILI9488_CMD_POSITIVE_GAMMA_CORRECTION       0xe0
#define ILI9488_CMD_NEGATIVE_GAMMA_CORRECTION       0xe1
#define ILI9488_CMD_DIGITAL_GAMMA_CONTROL_1         0xe2
#define ILI9488_CMD_DIGITAL_GAMMA_CONTROL_2         0xe3
#define ILI9488_CMD_SET_IMAGE_FUNCTION              0xe9
#define ILI9488_CMD_ADJUST_CONTROL_2                0xf2
#define ILI9488_CMD_ADJUST_CONTROL_3                0xf7
#define ILI9488_CMD_ADJUST_CONTROL_4                0xf8
#define ILI9488_CMD_ADJUST_CONTROL_5                0xf9
#define ILI9488_CMD_SPI_READ_SETTINGS               0xfb
#define ILI9488_CMD_ADJUST_CONTROL_6                0xfc
#define ILI9488_CMD_ADJUST_CONTROL_7                0xff

#endif /* __INCLUDE_NUTTX_LCD_ILI9488_H */

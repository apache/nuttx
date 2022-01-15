/****************************************************************************
 * arch/arm/src/lpc43xx/spifi/inc/private/spifilib_chiphw.h
 *
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licenser disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no
 * responsibility or liability for the use of the software, conveys no
 * license or rights under any patent, copyright, mask work right, or any
 * other intellectual property rights in or to any products. NXP
 * Semiconductors reserves the right to make changes in the software without
 * notification. NXP Semiconductors also makes no representation or warranty
 * that such application will be suitable for the specified use without
 * further testing or modification.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that
 * it is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC43XX_SPIFI_INC_PRIVATE_SPIFILIB_CHIPHW_H
#define __ARCH_ARM_SRC_LPC43XX_SPIFI_INC_PRIVATE_SPIFILIB_CHIPHW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Define for inline */
#ifndef INLINE
#ifdef __CC_ARM
#define INLINE  __inline
#else
#define INLINE inline
#endif /* __CC_ARM */
#endif /* !INLINE */

#ifdef __CC_ARM
#pragma anon_unions
#endif

/* LPCSPIFILIB_HW_API LPCSPIFILIB hardware definitions and API functions */

/* SPIFI controller hardware register structure */

typedef struct LPC_SPIFI_CHIPHW
{
  volatile    uint32_t CTRL;        /* SPIFI control register */
  volatile    uint32_t CMD;         /* SPIFI command register */
  volatile    uint32_t ADDR;        /* SPIFI address register */
  volatile    uint32_t DATINTM;     /* SPIFI intermediate data register */
  volatile    uint32_t CACHELIMIT;  /* SPIFI cache limit register */
  union
    {
      volatile    uint8_t DAT8;     /* SPIFI 8 bit data */
      volatile    uint16_t DAT16;   /* SPIFI 16 bit data */
      volatile    uint32_t DAT32;   /* SPIFI 32 bit data */
    };
  volatile    uint32_t MEMCMD;      /* SPIFI memory command register */
  volatile    uint32_t STAT;        /* SPIFI status register */
} LPC_SPIFI_CHIPHW_T;

/* LPCSPIFILIB_HW_PRIM LPCSPIFILIB primitive API functions */

/* SPIFI controller control register bit definitions */

#define SPIFI_CTRL_TO(t)        ((t) << 0)              /* SPIFI timeout */
#define SPIFI_CTRL_CSHI(c)      ((c) << 16)             /* SPIFI chip select minimum high time */
#define SPIFI_CTRL_DATA_PREFETCH_DISABLE(d) ((d) << 21) /* SPIFI memMode prefetch enable*/
#define SPIFI_CTRL_INTEN(i)     ((i) << 22)             /* SPIFI cmdComplete irq enable */
#define SPIFI_CTRL_MODE3(m)     ((m) << 23)             /* SPIFI mode3 config */
#define SPIFI_CTRL_PREFETCH_DISABLE(d) ((d) << 27)      /* SPIFI cache prefetch enable */
#define SPIFI_CTRL_DUAL(d)      ((d) << 28)             /* SPIFI enable dual */
#define SPIFI_CTRL_RFCLK(m)     ((m) << 29)             /* SPIFI clock edge config */
#define SPIFI_CTRL_FBCLK(m)     ((m) << 30)             /* SPIFI feedback clock select */
#define SPIFI_CTRL_DMAEN(m)     ((m) << 31)             /* SPIFI dma enable */

/* Write SPIFI controller control register
 * pSpifi  : Base address of SPIFI controller
 * ctrl    : Control value to write
 * Nothing
 */

static INLINE void spifi_HW_SetCtrl(LPC_SPIFI_CHIPHW_T *pSpifi,
                                    uint32_t ctrl)
{
  pSpifi->CTRL = ctrl;
}

/* Read SPIFI controller control register
 * pSpifi : Base address of SPIFI controller
 * Current CTRL register values
 */

static INLINE uint32_t spifi_HW_GetCtrl(LPC_SPIFI_CHIPHW_T *pSpifi)
{
  return pSpifi->CTRL;
}

/* SPIFI controller status register bit definitions */

#define SPIFI_STAT_RESET        (1 << 4)     /* SPIFI reset */
#define SPIFI_STAT_INTRQ        (1 << 5)     /* SPIFI interrupt request */
#define SPIFI_STAT_CMD          (1 << 1)     /* SPIFI command in progress */
#define SPIFI_STAT_MCINIT       (1)          /* SPIFI MCINIT */

/* Write SPIFI controller status register
 * pSpifi  : Base address of SPIFI controller
 * stat    : Status bits to write
 * Nothing
 */

static INLINE void spifi_HW_SetStat(LPC_SPIFI_CHIPHW_T *pSpifi,
                                    uint32_t stat)
{
  pSpifi->STAT = stat;
}

/* Read SPIFI controller status register
 * pSpifi  : Base address of SPIFI controller
 * Current STAT register values
 */

static INLINE uint32_t spifi_HW_GetStat(LPC_SPIFI_CHIPHW_T *pSpifi)
{
  return pSpifi->STAT;
}

/* SPIFI controller command register bit definitions */

#define SPIFI_CMD_DATALEN(l)    ((l) << 0)             /* SPIFI bytes to send or receive */
#define SPIFI_CMD_POLLRS(p)     ((p) << 14)            /* SPIFI enable poll */
#define SPIFI_CMD_DOUT(d)       ((d) << 15)            /* SPIFI data direction is out */
#define SPIFI_CMD_INTER(i)      ((i) << 16)            /* SPIFI intermediate bit length */
#define SPIFI_CMD_FIELDFORM(p)  ((p) << 19)            /* SPIFI 2 bit data/cmd mode control */
#define SPIFI_CMD_FRAMEFORM(f)  ((f) << 21)            /* SPIFI op and adr field config */
#define SPIFI_CMD_OPCODE(o)     ((uint32_t) (o) << 24) /* SPIFI 8-bit command code */

/* frame form definitions */

typedef enum
{
  SPIFI_FRAMEFORM_OP              = 1,
  SPIFI_FRAMEFORM_OP_1ADDRESS     = 2,
  SPIFI_FRAMEFORM_OP_2ADDRESS     = 3,
  SPIFI_FRAMEFORM_OP_3ADDRESS     = 4,
  SPIFI_FRAMEFORM_OP_4ADDRESS     = 5,
  SPIFI_FRAMEFORM_NOOP_3ADDRESS   = 6,
  SPIFI_FRAMEFORM_NOOP_4ADDRESS   = 7
} SPIFI_FRAMEFORM_T;

/* serial type definitions */

typedef enum
{
  SPIFI_FIELDFORM_ALL_SERIAL             = 0,
  SPIFI_FIELDFORM_SERIAL_OPCODE_ADDRESS  = 1,
  SPIFI_FIELDFORM_SERIAL_OPCODE          = 2,
  SPIFI_FIELDFORM_NO_SERIAL              = 3
} SPIFI_FIELDFORM_T;

/* Read SPIFI controller command register
 * pSpifi  : Base address of SPIFI controller
 * 32-bit value read from the command register
 */

static INLINE uint32_t spifi_HW_GetCmd(LPC_SPIFI_CHIPHW_T *pSpifi)
{
  return pSpifi->CMD;
}

/* Write SPIFI controller command register
 * pSpifi  : Base address of SPIFI controller
 * cmd     : Command to write
 * return Nothing
 */

static INLINE void spifi_HW_SetCmd(LPC_SPIFI_CHIPHW_T *pSpifi,
                                   uint32_t cmd)
{
  pSpifi->CMD = cmd;
}

/* Write SPIFI controller address register
 * pSpifi  : Base address of SPIFI controller
 * addr    : address (offset) to write
 * return Nothing
 */

static INLINE void spifi_HW_SetAddr(LPC_SPIFI_CHIPHW_T *pSpifi,
                                    uint32_t addr)
{
  pSpifi->ADDR = addr;
}

/*   Read an 8-bit value from the controller data register
 *    pSpifi : Base address of SPIFI controller
 *  8-bit value read from the data register
 */

static INLINE uint8_t spifi_HW_GetData8(LPC_SPIFI_CHIPHW_T *pSpifi)
{
  return pSpifi->DAT8;
}

/* Read an 16-bit value from the controller data register
 * pSpifi  : Base address of SPIFI controller
 * 16-bit value read from the data register
 */

static INLINE uint16_t spifi_HW_GetData16(LPC_SPIFI_CHIPHW_T *pSpifi)
{
  return pSpifi->DAT16;
}

/* Read an 32-bit value from the controller data register
 * pSpifi : Base address of SPIFI controller
 * 32-bit value read from the data register
 */

static INLINE uint32_t spifi_HW_GetData32(LPC_SPIFI_CHIPHW_T *pSpifi)
{
  return pSpifi->DAT32;
}

/* Write an 8-bit value from the controller data register
 * pSpifi  : Base address of SPIFI controller
 * data    : 8-bit data value to write
 * return   Nothing
 */

static INLINE void spifi_HW_SetData8(LPC_SPIFI_CHIPHW_T *pSpifi,
                                     uint8_t data)
{
  pSpifi->DAT8 = data;
}

/* Write an 16-bit value from the controller data register
 * pSpifi  : Base address of SPIFI controller
 * data    : 16-bit data value to write
 * return Nothing
 */

static INLINE void spifi_HW_SetData16(LPC_SPIFI_CHIPHW_T *pSpifi,
                                      uint16_t data)
{
  pSpifi->DAT16 = data;
}

/* Write an 32-bit value from the controller data register
 * pSpifi  : Base address of SPIFI controller
 * data    : 32-bit data value to write
 * return Nothing
 */

static INLINE void spifi_HW_SetData32(LPC_SPIFI_CHIPHW_T *pSpifi,
                                      uint32_t data)
{
  pSpifi->DAT32 = data;
}

/* Write IDATA register
 * pSpifi : Base address of SPIFI controller
 * mode : value to write. Used to specify value used for intermediate
 *                                   data value when enabled.
 * return Nothing
 */

static INLINE void spifi_HW_SetIDATA(LPC_SPIFI_CHIPHW_T *pSpifi,
                                      uint32_t mode)
{
  pSpifi->DATINTM = mode;
}

/* Write MEMCMD register
 * pSpifi : Base address of SPIFI controller
 * cmd     : Command value to write
 * return Nothing
 */

static INLINE void spifi_HW_SetMEMCMD(LPC_SPIFI_CHIPHW_T *pSpifi,
                                      uint32_t cmd)
{
  pSpifi->MEMCMD = cmd;
}

/* LPCSPIFILIB_HW_L2 LPCSPIFILIB hardware support API functions */

/* Reset SPIFI controller
 * pSpifi  : Base address of SPIFI controller
 * return Nothing
 */

static INLINE void spifi_HW_ResetController(LPC_SPIFI_CHIPHW_T *pSpifi)
{
  pSpifi->STAT = SPIFI_STAT_RESET;
  while ((pSpifi->STAT & SPIFI_STAT_RESET) != 0)
    {
    }
}

/* Wait for a command to complete
 * pSpifi  : Base address of SPIFI controller
 * return Nothing
 */

static INLINE void spifi_HW_WaitCMD(LPC_SPIFI_CHIPHW_T *pSpifi)
{
  while ((spifi_HW_GetStat(pSpifi) & SPIFI_STAT_CMD) != 0)
    {
    }
}

/* Wait for a RESET bit to clear
 * pSpifi  : Base address of SPIFI controller
 * return Nothing
 */

static INLINE void spifi_HW_WaitRESET(LPC_SPIFI_CHIPHW_T *pSpifi)
{
  while ((spifi_HW_GetStat(pSpifi) & SPIFI_STAT_RESET) != 0)
    {
    }
}

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_LPC43XX_SPIFI_INC_PRIVATE_SPIFILIB_CHIPHW_H */

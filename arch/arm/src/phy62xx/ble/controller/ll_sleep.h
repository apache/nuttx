/**************************************************************************************************

    Phyplus Microelectronics Limited confidential and proprietary.
    All rights reserved.

    IMPORTANT: All rights of this software belong to Phyplus Microelectronics
    Limited ("Phyplus"). Your use of this Software is limited to those
    specific rights granted under  the terms of the business contract, the
    confidential agreement, the non-disclosure agreement and any other forms
    of agreements as a customer or a partner of Phyplus. You may not use this
    Software unless you agree to abide by the terms of these agreements.
    You acknowledge that the Software may not be modified, copied,
    distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy
    (BLE) integrated circuit, either as a product or is integrated into your
    products.  Other than for the aforementioned purposes, you may not use,
    reproduce, copy, prepare derivative works of, modify, distribute, perform,
    display or sell this Software and/or its documentation for any purposes.

    YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
    PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
    INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
    NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
    PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
    NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
    LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
    OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
    OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

**************************************************************************************************/


#ifndef LL_SLEEP__H_
#define LL_SLEEP__H_

#include "OSAL_PwrMgr.h"

#include "ll_def.h"
#include "ll_common.h"

/*******************************************************************************
    MACROS
*/

// convert 625us units to 32kHz units without round: the ratio of 32 kHz ticks
// to 625 usec ticks is 32768/1600 = 20.48 or 512/25
#define LL_SLEEP_625US_TO_32KHZ( us )       ((((uint32) (us)) * 512) / 25)

// convert 31.25ns units to 32kHz units without round: the ratio of 31.25ns usec
// ticks to 32 kHz ticks is 32M/32768 = 976.5625 or 15625/16, but using 976 is
// close enough given the accuracy
#define LL_SLEEP_31_25NS_TO_32KHZ( ns )     (((uint32) (ns)) / 976)


// 32KHz timer:
//  crystal: 32768Hz
//  RC     : 32768Hz Should be same as Xtal
//  timer1 - 4 : 4MHz
#define TIMER_TO_32K_CRYSTAL          122           //  122.0703
#define TIMER_TO_32K_RC               122           //  125

#define   STD_RC32_8_CYCLE_16MHZ_CYCLE     3906             // standard 16Mhz cycles for 8 RC32KHz tick
#define   STD_CRY32_8_CYCLE_16MHZ_CYCLE    3906             // standard 16Mhz cycles for 8 crystal 32KHz tick
#define   ERR_THD_RC32_CYCLE                200             // error threshold for N+x rcosc tracking cycle


#define   CRY32_8_CYCLE_16MHZ_CYCLE_MAX    (3906 + 196)     // tracking value range std +/- 5%
#define   CRY32_8_CYCLE_16MHZ_CYCLE_MIN    (3906 - 196)

#define   STD_RC32_16_CYCLE_16MHZ_CYCLE     (7812)          // standard 16Mhz cycles for 16 RC32KHz tick
#define   STD_CRY32_16_CYCLE_16MHZ_CYCLE    (7812)          // standard 16Mhz cycles for 16 crystal 32KHz tick


#define   CRY32_16_CYCLE_16MHZ_CYCLE_MAX    (7812 + 391)     // tracking value range std +/- 5%
#define   CRY32_16_CYCLE_16MHZ_CYCLE_MIN    (7812 - 391)

#define   SLEEP_MAGIC                 0x032141B6


/*******************************************************************************
    TYPEDEFS
*/
typedef enum
{
    MCU_SLEEP_MODE,
    SYSTEM_SLEEP_MODE,
    SYSTEM_OFF_MODE
}  Sleep_Mode;



/*******************************************************************************
    Functions
*/

// is sleep allow
uint8 isSleepAllow(void);

void enableSleep(void);

void disableSleep(void);

void setSleepMode(Sleep_Mode mode);

Sleep_Mode getSleepMode(void);

void enterSleepProcess(uint32 time);

void wakeupProcess(void);

void set_sleep_flag(int flag);

unsigned int get_sleep_flag(void);

void config_RTC(uint32 time);

void enter_sleep_off_mode(Sleep_Mode mode);

#endif     // LL_SLEEP__H_





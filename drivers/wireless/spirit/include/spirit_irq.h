/******************************************************************************
 * include/nuttx/wireless/spirit/include/spirit_irq.h
 * Configuration and management of SPIRIT IRQs.
 *
 *   Copyright(c) 2015 STMicroelectronics
 *   Author: VMA division - AMS
 *   Version 3.2.2 08-July-2015
 *
 *   Adapted for NuttX by:
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#ifndef __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_IRQ_H
#define __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_IRQ_H

/* On the Spirit side specific IRQs can be enabled by setting a specific bitmask.
 * The Spirit libraries allow the user to do this in two different ways:
 *
 * The first enables the IRQs one by one, i.e. using an SPI transaction for each
 * IRQ to enable.
 *
 * Example:
 *
 *   spirit_irq_disableall(spirit); # this call is used to reset the IRQ mask registers
 *   spirit_irq_enable(spirit, RX_DATA_READY , S_ENABLE);
 *   spirit_irq_enable(spirit, VALID_SYNC , S_ENABLE);
 *   spirit_irq_enable(spirit, RX_TIMEOUT , S_ENABLE);
 *
 * The most applications will require a Spirit IRQ notification on an
 * microcontroller EXTI line.  Then, the user can check which IRQ has been
 * raised using two different ways.
 *
 * On the ISR of the EXTI line phisically linked to the Spirit pin
 * configured for IRQ:
 *
 * Check only one Spirit IRQ (because the Spirit IRQ status register
 * automatically blanks itself after an SPI reading) into the ISR.
 *
 * Example:
 *
 *   if (spirit_irq_is_pending(RX_DATA_READY))
 *     {
 *       # do something...
 *     }
 *
 * Check more than one Spirit IRQ status by storing the entire IRQ status
 * registers into a bitfields struct spirit_irqset_s structure and then
 * check the interesting bits.
 *
 * Example:
 *
 *   spirit_irq_get_pending(&irqStatus);
 *
 *   if (irqStatus.IRQ_RX_DATA_READY)
 *     {
 *       # do something...
 *     }
 *
 *   if (irqStatus.IRQ_VALID_SYNC)
 *     {
 *       # do something...
 *     }
 *
 *   if (irqStatus.RX_TIMEOUT)
 *     {
 *       # do something...
 *     }
 */

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include "spirit_types.h"
#include "spirit_regs.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* uint32_t masks */

#define IRQ_TX_FIFO_ALMOST_EMPTY_MASK  (0x00010000)  /* (1 << 16) */
#define IRQ_RX_FIFO_ALMOST_FULL_MASK   (0x00020000)  /* (1 << 17) */
#define IRQ_VALID_SYNC_MASK            (0x00200000)  /* (1 << 21) */
#define IRQ_RX_DATA_READY_MASK         (0x01000000)  /* (1 << 24) */
#define IRQ_RX_DATA_DISC_MASK          (0x02000000)  /* (1 << 25) */
#define IRQ_TX_DATA_SENT_MASK          (0x04000000)  /* (1 << 26) */
#define IRQ_TX_FIFO_ERROR_MASK         (0x20000000)  /* (1 << 29) */
#define IRQ_RX_FIFO_ERROR_MASK         (0x40000000)  /* (1 << 30) */

/* Macros used in assertions */

#define IS_SPIRIT_IRQ_LIST(value) \
  ((value == RX_DATA_READY)        || (value == RX_DATA_DISC)         || \
   (value == TX_DATA_SENT)         || (value == MAX_RE_TX_REACH)      || \
   (value == CRC_ERROR)            || (value == TX_FIFO_ERROR)        || \
   (value == RX_FIFO_ERROR)        || (value == TX_FIFO_ALMOST_FULL)  || \
   (value == TX_FIFO_ALMOST_EMPTY) || (value == RX_FIFO_ALMOST_FULL)  || \
   (value == RX_FIFO_ALMOST_EMPTY) || (value == MAX_BO_CCA_REACH)     || \
   (value == VALID_PREAMBLE)       || (value == VALID_SYNC)           || \
   (value == RSSI_ABOVE_TH)        || (value == WKUP_TOUT_LDC)        || \
   (value == READY)                || (value == STANDBY_DELAYED)      || \
   (value == LOW_BATT_LVL)         || (value == POR)                  || \
   (value == BOR)                  || (value == LOCK)                 || \
   (value == PM_COUNT_EXPIRED)     || (value == XO_COUNT_EXPIRED)     || \
   (value == SYNTH_LOCK_TIMEOUT)   || (value == SYNTH_LOCK_STARTUP)   || \
   (value == SYNTH_CAL_TIMEOUT)    || (value == TX_START_TIME)        || \
   (value == RX_START_TIME)        || (value == RX_TIMEOUT)           || \
   (value == AES_END)              || (value == ALL_IRQ ))

/******************************************************************************
 * Public Types
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* IRQ bitfield structure for SPIRIT. This structure is used to read or write
 * the single IRQ bit.  During the initialization the user has to fill this
 * structure setting to one the single field related to the IRQ he wants to
 * enable, and to zero the single field related to all the IRQs he wants to
 * disable.
 *
 * The same structure can be used to retrieve all the IRQ events from the IRQ
 * registers IRQ_STATUS[3:0], and read if one or more specific IRQ raised.
 *
 * NOTE:  The fields order in the structure depends on used endianness (little
 * or big  endian). The actual definition is valid ONLY for LITTLE ENDIAN
 * mode. Be sure to change the field order when use a different endianness.
 */

#ifndef CONFIG_ENDIAN_BIG
struct spirit_irqset_s
{
  uint8_t IRQ_SYNTH_LOCK_TIMEOUT   : 1;  /* IRQ: only for debug; LOCK state
                                          * timeout */
  uint8_t IRQ_SYNTH_LOCK_STARTUP   : 1;  /* IRQ: only for debug; see
                                          * CALIBR_START_COUNTER */
  uint8_t IRQ_SYNTH_CAL_TIMEOUT    : 1;  /* IRQ: only for debug; SYNTH
                                          * calibration timeout */
  uint8_t IRQ_TX_START_TIME        : 1;  /* IRQ: only for debug; TX
                                          * circuitry startup time; see
                                          * TX_START_COUNTER */
  uint8_t IRQ_RX_START_TIME        : 1;  /* IRQ: only for debug; RX
                                          * circuitry startup time; see
                                          * TX_START_COUNTER */
  uint8_t IRQ_RX_TIMEOUT           : 1;  /* IRQ: RX operation timeout */
  uint8_t IRQ_AES_END              : 1;  /* IRQ: AES End of operation */
  uint8_t reserved                 : 1;  /* Reserved bit */

  uint8_t IRQ_READY                : 1;  /* IRQ: READY state */
  uint8_t IRQ_STANDBY_DELAYED      : 1;  /* IRQ: STANDBY state after
                                          * MCU_CK_CONF_CLOCK_TAIL_X
                                          * clock cycles */
  uint8_t IRQ_LOW_BATT_LVL         : 1;  /* IRQ: Battery level below
                                          * threshold */
  uint8_t IRQ_POR                  : 1;  /* IRQ: Power On Reset */
  uint8_t IRQ_BOR                  : 1;  /* IRQ: Brown out event (both
                                          * accurate and inaccurate) */
  uint8_t IRQ_LOCK                 : 1;  /* IRQ: LOCK state */
  uint8_t IRQ_PM_COUNT_EXPIRED     : 1;  /* IRQ: only for debug;
                                          * Power Management startup
                                          * timer expiration (see reg
                                          * PM_START_COUNTER, 0xB5) */
  uint8_t IRQ_XO_COUNT_EXPIRED     : 1;  /* IRQ: only for debug;
                                          * Crystal oscillator settling
                                          * time counter expired */

  uint8_t IRQ_TX_FIFO_ALMOST_EMPTY : 1;  /* IRQ: TX FIFO almost empty */
  uint8_t IRQ_RX_FIFO_ALMOST_FULL  : 1;  /* IRQ: RX FIFO almost full */
  uint8_t IRQ_RX_FIFO_ALMOST_EMPTY : 1;  /* IRQ: RX FIFO almost empty */
  uint8_t IRQ_MAX_BO_CCA_REACH     : 1;  /* IRQ: Max number of back-off
                                          * during CCA */
  uint8_t IRQ_VALID_PREAMBLE       : 1;  /* IRQ: Valid preamble detected */
  uint8_t IRQ_VALID_SYNC           : 1;  /* IRQ: Sync word detected */
  uint8_t IRQ_RSSI_ABOVE_TH        : 1;  /* IRQ: RSSI above threshold */
  uint8_t IRQ_WKUP_TOUT_LDC        : 1;  /* IRQ: Wake-up timeout in LDC mode */

  uint8_t IRQ_RX_DATA_READY        : 1;  /* IRQ: RX data ready */
  uint8_t IRQ_RX_DATA_DISC         : 1;  /* IRQ: RX data discarded
                                          * (upon filtering) */
  uint8_t IRQ_TX_DATA_SENT         : 1;  /* IRQ: TX data sent */
  uint8_t IRQ_MAX_RE_TX_REACH      : 1;  /* IRQ: Max re-TX reached */
  uint8_t IRQ_CRC_ERROR            : 1;  /* IRQ: CRC error */
  uint8_t IRQ_TX_FIFO_ERROR        : 1;  /* IRQ: TX FIFO underflow/overflow
                                          * error */
  uint8_t IRQ_RX_FIFO_ERROR        : 1;  /* IRQ: RX FIFO underflow/overflow
                                          * error */
  uint8_t IRQ_TX_FIFO_ALMOST_FULL  : 1;  /* IRQ: TX FIFO almost full */
};
#endif

/* IRQ list enumeration for SPIRIT. This enumeration type can be used to
 * address a specific IRQ.
 */

enum spirit_irq_e
{
  RX_DATA_READY        = 0x00000001,  /* IRQ: RX data ready */
  RX_DATA_DISC         = 0x00000002,  /* IRQ: RX data discarded (upon
                                       * filtering) */
  TX_DATA_SENT         = 0x00000004,  /* IRQ: TX data sent */
  MAX_RE_TX_REACH      = 0x00000008,  /* IRQ: Max re-TX reached */
  CRC_ERROR            = 0x00000010,  /* IRQ: CRC error */
  TX_FIFO_ERROR        = 0x00000020,  /* IRQ: TX FIFO underflow/overflow
                                       * error */
  RX_FIFO_ERROR        = 0x00000040,  /* IRQ: RX FIFO underflow/overflow
                                       * error */
  TX_FIFO_ALMOST_FULL  = 0x00000080,  /* IRQ: TX FIFO almost full */
  TX_FIFO_ALMOST_EMPTY = 0x00000100,  /* IRQ: TX FIFO almost empty */
  RX_FIFO_ALMOST_FULL  = 0x00000200,  /* IRQ: RX FIFO almost full */
  RX_FIFO_ALMOST_EMPTY = 0x00000400,  /* IRQ: RX FIFO almost empty */
  MAX_BO_CCA_REACH     = 0x00000800,  /* IRQ: Max number of back-off
                                       * during CCA */
  VALID_PREAMBLE       = 0x00001000,  /* IRQ: Valid preamble detected */
  VALID_SYNC           = 0x00002000,  /* IRQ: Sync word detected */
  RSSI_ABOVE_TH        = 0x00004000,  /* IRQ: RSSI above threshold */
  WKUP_TOUT_LDC        = 0x00008000,  /* IRQ: Wake-up timeout in LDC mode */
  READY                = 0x00010000,  /* IRQ: READY state */
  STANDBY_DELAYED      = 0x00020000,  /* IRQ: STANDBY state after
                                       * MCU_CK_CONF_CLOCK_TAIL_X clock
                                       * cycles */
  LOW_BATT_LVL         = 0x00040000,  /* IRQ: Battery level below
                                       * threshold */
  POR                  = 0x00080000,  /* IRQ: Power On Reset */
  BOR                  = 0x00100000,  /* IRQ: Brown out event (both accurate and
                                       * inaccurate) */
  LOCK                 = 0x00200000,  /* IRQ: LOCK state */
  PM_COUNT_EXPIRED     = 0x00400000,  /* IRQ: only for debug; Power
                                       * Management startup timer expiration
                                       * (see reg PM_START_COUNTER, 0xB5) */
  XO_COUNT_EXPIRED     = 0x00800000,  /* IRQ: only for debug; Crystal
                                       * oscillator settling time counter
                                       * expired */
  SYNTH_LOCK_TIMEOUT   = 0x01000000,  /* IRQ: only for debug; LOCK state
                                       * timeout */
  SYNTH_LOCK_STARTUP   = 0x02000000,  /* IRQ: only for debug; see
                                       * CALIBR_START_COUNTER */
  SYNTH_CAL_TIMEOUT    = 0x04000000,  /* IRQ: only for debug; SYNTH
                                       * calibration timeout */
  TX_START_TIME        = 0x08000000,  /* IRQ: only for debug; TX circuitry
                                      * startup time; see TX_START_COUNTER */
  RX_START_TIME        = 0x10000000,  /* IRQ: only for debug; RX circuitry
                                      * startup time; see TX_START_COUNTER */
  RX_TIMEOUT           = 0x20000000,  /* IRQ: RX operation timeout */
  AES_END              = 0x40000000,  /* IRQ: AES End of operation */
  ALL_IRQ              = 0x7fffffff   /* All the above mentioned IRQs */
};

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_irq_disable_all
 *
 * Description:
 *   Ssets the IRQ mask registers to 0x00000000, disabling all IRQs.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_irq_disable_all(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_irq_set_mask
 *
 * Description:
 *   Enables/disables all the IRQs according to the user defined irqset
 *   structure.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   irqset - Pointer to a variable of type struct spirit_irqset_s, through
 *            which the user enable specific IRQs. This parameter is a
 *            pointer to a struct spirit_irqset_s.
 *
 *            For example suppose to enable only the two IRQ Low Battery Level
 *            and Tx Data Sent:
 *
 *              struct spirit_irqset_s g_irqset = {0};
 *              g_irqset.IRQ_LOW_BATT_LVL = 1;
 *              g_irqset.IRQ_TX_DATA_SENT = 1;
 *              spirit_irq_setmask(&g_irqset);
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

#ifndef CONFIG_ENDIAN_BIG
int spirit_irq_set_mask(FAR struct spirit_library_s *spirit,
                        FAR struct spirit_irqset_s *irqset);
#endif

/******************************************************************************
 * Name: spirit_irq_enable
 *
 * Description:
 *   Enables or disables a specific IRQ.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   irq IRQ to enable or disable.
 *   newstate - new state for the IRQ.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_irq_enable(FAR struct spirit_library_s *spirit,
                      enum spirit_irq_e irq,
                      enum spirit_functional_state_e newstate);

/******************************************************************************
 * Name: spirit_irt_get_mask
 *
 * Description:
 *   Fills a pointer to a structure of struct spirit_irqset_s type with the
 *   content of the IRQ_MASK registers.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   pirqmask - Pointer to a variable of type struct spirit_irqset_s, through
 *              which the user can read which IRQs are enabled. All the
 *              bitfields equals to zero correspond to enabled IRQs, while
 *              all the bitfields equals to one correspond to disabled IRQs.
 *
 *              For example suppose that the Power On Reset and RX Data
 *              ready are the only enabled IRQs.
 *
 *                struct spirit_irqset_s g_irqmask;
 *                spirit_irq_get_pending(&g_irqmask);
 *
 *              Then g_irqmask.IRQ_POR and g_irqmask.IRQ_RX_DATA_READY are
 *              equal to 0 while all the other bitfields are equal to one.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

#ifndef CONFIG_ENDIAN_BIG
int spirit_irt_get_mask(FAR struct spirit_library_s *spirit,
                        FAR struct spirit_irqset_s *pirqmask);
#endif

/******************************************************************************
 * Name: spirit_irq_get_pending
 *
 * Description:
 *   Fills a pointer to a structure of struct spirit_irqset_s type with the
 *   content of the IRQ_STATUS registers.
 *
 * Input Parameters:
 *   spirit     - Reference to a Spirit library state structure instance
 *   pirqstatus - A pointer to a variable of type struct spirit_irqset_s,
 *                through which the user can receive the status of all the
 *                IRQs. All the bitfields equals to one correspond to the
 *                raised interrupts.
 *
 *                For example suppose that the XO settling timeout is raised
 *                as well as the Sync word detection.
 *
 *                  struct spirit_irqset_s g_irqstatus;
 *                  spirit_irq_get_pending(&g_irqstatus);
 *
 *                Then g_irqstatus.IRQ_XO_COUNT_EXPIRED and
 *                g_irqstatus.IRQ_VALID_SYNC are equals to 1* while all the
 *                other bitfields are equals to zero.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

#ifndef CONFIG_ENDIAN_BIG
int spirit_irq_get_pending(FAR struct spirit_library_s *spirit,
                           FAR struct spirit_irqset_s *pirqstatus);
#endif

/******************************************************************************
 * Name: spirit_irq_clr_pending
 *
 * Description:
 *   Clear all IRQ status registers.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_irq_clr_pending(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_irq_is_pending
 *
 * Description:
 *   Checks if a specific IRQ has been generated.  The call resets all the
 *   IRQ status, so it can't be used in case of multiple raising interrupts.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   flag IRQ flag to be checked.
 *         This parameter can be any value of enum spirit_irq_e.
 *
 * Returned Value:
 *   true or false.
 *
 ******************************************************************************/

bool spirit_irq_is_pending(FAR struct spirit_library_s *spirit,
                          enum spirit_irq_e flag);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_IRQ_H */

/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_emc.c
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>


/* TODO: add #if defined(CONFIG_LPC43_EMC) */

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <queue.h>
#include <errno.h>


#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>

#include "up_internal.h"

#include "chip.h"
#include "lpc43_pinconfig.h"
#include "lpc43_emc.h"
#include "chip/lpc43_creg.h"
#include "chip/lpc43_cgu.h"
#include "chip/lpc43_ccu.h"
#include "lpc43_rgu.h"
#include "lpc43_gpio.h"
#include "up_arch.h"
#include <arch/board/board.h>


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_emcinit
 *
 * Description:
 *   Initialize EMC controller. Start in full power
 *   mode.
 *
 ****************************************************************************/
void lpc43_emcinit(uint32_t enable, uint32_t clock_ratio, uint32_t endian_mode)
{
  uint32_t regval;

  /* Enable clock for EMC controller. */

  regval = getreg32(LPC43_CCU1_M4_EMC_CFG);
  regval |= CCU_CLK_CFG_RUN;
  putreg32(regval, LPC43_CCU1_M4_EMC_CFG);

  /* Configure endian mode and clock ratio. */

  regval = 0;
  if (endian_mode)
    regval |= EMC_CONFIG_EM;
  if (clock_ratio)
    regval |= EMC_CONFIG_CR;

  putreg32(regval, LPC43_EMC_CONFIG);

  /* Enable EMC 001 normal memory map, no low power mode. */

  putreg32(EMC_CONTROL_ENA, LPC43_EMC_CONTROL);
}

/****************************************************************************
 * Name: lpc43_lowpowermode
 *
 * Description:
 *   Set EMC lowpower mode.
 *
 ****************************************************************************/
void lpc43_lowpowermode(uint8_t enable)
{
  uint32_t regval;

  regval = getreg32(LPC43_EMC_CONTROL);
  if (enable)
    {
      regval |= EMC_CONTROL_LOWPOWER;
      putreg32(regval, LPC43_EMC_CONTROL);
    }
  else
    {
      regval &= ~EMC_CONTROL_LOWPOWER;
      putreg32(regval, LPC43_EMC_CONTROL);
    }
}

/****************************************************************************
 * Name: lpc43_emcenable
 *
 * Description:
 *   Enable or disable EMC controller.
 *
 ****************************************************************************/
void lpc43_emcenable(uint8_t enable)
{
  uint32_t regval;

  regval = getreg32(LPC43_EMC_CONTROL);
  if (enable)
    {
      regval |= EMC_CONTROL_ENA;
      putreg32(regval, LPC43_EMC_CONTROL);
    }
  else
    {
      regval &= ~EMC_CONTROL_ENA;
      putreg32(regval, LPC43_EMC_CONTROL);
    }
}

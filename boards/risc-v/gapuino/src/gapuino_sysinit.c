/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include "gap8.h"
#include "gap8_fll.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Used to communicate with plpbridge */

struct _debug_struct
{
  /* Used by external debug bridge to get exit status when using the board */

  uint32_t exitStatus;

  /* Printf */

  uint32_t useInternalPrintf;
  uint32_t putcharPending;
  uint32_t putcharCurrent;
  uint8_t putcharBuffer[128];

  /* Debug step, used for showing progress to host loader */

  uint32_t debugStep;
  uint32_t debugStepPending;

  /* Requests */

  uint32_t firstReq;
  uint32_t lastReq;
  uint32_t firstBridgeReq;

  uint32_t notifReqAddr;
  uint32_t notifReqValue;

  uint32_t bridgeConnected;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Place a dummy debug struct */

struct _debug_struct Debug_Struct =
{
  .useInternalPrintf = 1,
};

uint32_t g_current_freq = 0;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gapuino_sysinit
 *
 * Description:
 *   Initialize cache, clock, etc.
 *
 ****************************************************************************/

void gapuino_sysinit(void)
{
  SCBC->ICACHE_ENABLE = 0xFFFFFFFF;
  gap8_setfreq(CONFIG_CORE_CLOCK_FREQ);

  /* For debug usage */

  g_current_freq = gap8_getfreq();
}

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include <nuttx/board.h>

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/* Application initialization stub for boardctl() */

#ifdef CONFIG_LIB_BOARDCTL
int board_app_initialize(void)
{
  return OK;
}
#endif /* CONFIG_LIB_BOARDCTL */

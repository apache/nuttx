/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>

#include "calypso_spi.h"

/****************************************************************************
 * Name: board_power_off
 *
 * Description:
 *   Power off the board.
 *
 *   If this function returns, then it was not possible to power-off the
 *   board due to some other constraints.
 *
 * Input Parameters:
 *   status - Status information provided with the power off event.
 *
 * Returned Value:
 *   If this function returns, then it was not possible to power-off the
 *   board due to some constraints.  The return value int this case is a
 *   board-specific reason for the failure to shutdown.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_POWEROFF
int board_power_off(int status)
{
  struct spi_dev_s *spi = calypso_spibus_initialize(0);
  uint16_t tx;

  SPI_SETBITS(spi, 16);
  (void)SPI_HWFEATURES(spi, 0);

  tx = (1 << 6) | (1 << 1);
  SPI_SNDBLOCK(spi, &tx, 1);

  tx = (1 << 6) | (30 << 1);
  SPI_SNDBLOCK(spi, &tx, 1);

  return 0;
}
#endif

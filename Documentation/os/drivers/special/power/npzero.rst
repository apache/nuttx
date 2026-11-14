=====================================
Nanopower nPZero G1S power-saving IC
=====================================

The nPZero G1S (NPZG1SQ32) from Nanopower Semiconductor is a power-saving IC
for sensor systems. While the host MCU is powered, the nPZero is an I2C slave
of the host (*Standby* state). When the host sends the *Idle* command, the
nPZero switches the host off and polls up to four I2C or SPI peripherals by
itself, powering them, sending them initialization commands from its 128-byte
SRAM, reading their values and comparing them with thresholds. When a wake-up
condition happens, it switches the host on again. The wake-up conditions are:

- a peripheral value inside the trigger area, or an interrupt of a peripheral,
- a VBAT or ``ADC_IN`` value outside the ADC thresholds,
- a NAK of an I2C peripheral (if enabled),
- the global time-out.

So the host boots at each wake-up. The usual program reads the wake-up reason,
handles it, sends the configuration and enters Idle state again.

The driver (``drivers/power/supply/npzero.c``) is a character driver controlled
with ``ioctl()`` commands. The register map, the values of the fields and the
commands are in ``include/nuttx/power/npzero.h``.

Configuration
=============

- ``CONFIG_NPZERO``: enables the driver. It needs ``CONFIG_I2C``.
- ``CONFIG_NPZERO_I2C_FREQUENCY``: I2C frequency, 100 kHz by default. The
  nPZero supports the I2C standard mode only.

Board interface
===============

The board registers the driver with::

  int npzero_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                      uint8_t addr);

``addr`` is the 7-bit I2C address, ``NPZERO_I2C_ADDR`` (0x3d). The function
reads the ID register and returns ``-ENODEV`` if it is not ``NPZERO_ID``
(0x60). Since the host is switched off in Idle state, it must be called at each
boot, for instance from ``board_late_initialize()``::

  #include <nuttx/power/npzero.h>

  i2c = stm32_i2cbus_initialize(1);
  ret = npzero_register("/dev/npzero0", i2c, NPZERO_I2C_ADDR);

Shared I2C bus
==============

The I2C bus of the host is also the bus of the I2C peripherals, and the
peripherals are usually powered by the ``SW_LP`` switches of the nPZero.
After a reset of the nPZero (power-on, brown-out, ``RSTN`` or soft reset),
all the peripheral switches are off. An unpowered peripheral can load SDA
through its protection diodes, which then rises too slowly for the I2C
controller of the host.

On the nPZero G1S Development Kit, an unpowered AS6212 temperature sensor
board makes SDA rise in about 10 us (100 us when SCL is low): the STM32 I2C
controller reports an arbitration loss and ``npzero_register()`` fails with
``-EAGAIN``. The Nanopower firmware fails in the same way. So:

- Set the ``pswint`` bits of the I2C peripherals in the configuration, so
  that they stay powered while the host is on (Standby state).
- After a reset of the nPZero, the board must enable these switches (PSWCTL)
  before it talks to the nPZero at 100 kHz. The ``npzero-g1s-dk`` board does
  it with a slow bit-banged I2C transfer, see
  :doc:`/platforms/arm/stm32l0/boards/npzero-g1s-dk/index`.
- Do not plug a sensor board while the host is running.

Errata
======

The driver applies the workarounds of the errata NPZ-G1S-ERRATA-V3.0:

- **#2, sequential SRAM writes can corrupt the SRAM**: ``NPZEROIOC_CONFIGURE``
  writes the whole SRAM one byte at a time, in the order of the workaround.
  ``NPZEROIOC_WRITEREG`` refuses the SRAM addresses (``-EACCES``).
- **#11, incorrect IREF trim values** (unexpected reset in Idle state):
  ``npzero_register()`` corrects the trim values when no wake-up flag is set
  in STA1 and STA2, that is after a reset of the nPZero.

The other errata are for the application:

- **#3**: an interrupt pulse shorter than 2.5 us wakes the nPZero up without
  setting its ``FP`` bit in STA2.
- **#4**: after a NAK (``FNAKP`` bit in STA2) or an interrupt time-out, the
  VALP registers of a peripheral may keep the value of another peripheral.
- **#5**: only the soft reset bit (``NPZERO_STA1_RSTSRC_SOFT``) of the reset
  source is reliable.
- **#6, #7**: for an 8-bit value, two bytes are read from the peripheral.
- **#9**: with the ``ADC_IN`` wake-up enabled, ``ADC_IN`` must stay between
  600 mV and VBAT.

The datasheet also forbids the peripheral power switch mode 0 (switch with
output voltage rise detection), which can lock up the polling, so the driver
refuses it.

IOCTL commands
==============

======================== ================================= ===================
Command                  Argument                          Description
======================== ================================= ===================
``NPZEROIOC_GETID``      ``FAR uint8_t *``                 Read the ID
                                                           register (0x60)
``NPZEROIOC_GETSTATUS``  ``FAR struct npzero_status_s *``  Read STA1 and STA2
``NPZEROIOC_CONFIGURE``  ``FAR const struct                Write the
                         npzero_config_s *``               configuration and
                                                           the SRAM
``NPZEROIOC_GETVALUE``   ``FAR struct npzero_value_s *``   Last value read
                                                           from a peripheral
``NPZEROIOC_GETADC``     ``FAR struct npzero_adc_s *``     Last ADC values
``NPZEROIOC_IDLE``       None                              Enter Idle state
``NPZEROIOC_RESET``      None                              Soft reset
``NPZEROIOC_READREG``    ``FAR struct npzero_reg_s *``     Read a register
``NPZEROIOC_WRITEREG``   ``FAR const struct                Write a register
                         npzero_reg_s *``                  (not the SRAM)
======================== ================================= ===================

Status
------

``struct npzero_status_s`` holds the raw ``sta1`` and ``sta2`` registers.
Use the ``NPZERO_STA1_*`` bits (``FTOUT`` global time-out, ``FA1`` VBAT
trigger, ``FA2`` ``ADC_IN`` trigger, ``RSTSRC`` reset source) and the
``NPZERO_STA2_FP(n)`` (trigger) and ``NPZERO_STA2_FNAKP(n)`` (NAK) bits of
peripheral ``n``.

``NPZEROIOC_GETVALUE`` returns the 16-bit ``value`` of the peripheral
``periph`` (1 to 4) in the type set by its ``dtype``. ``NPZEROIOC_GETADC``
returns the ADC codes, see the table *ADC codes* of the datasheet. The codes
of the VBAT channel are the ones of the ``ADC_IN`` channel minus 0x20.

Configuration structure
-----------------------

``NPZEROIOC_CONFIGURE`` takes a ``struct npzero_config_s``. It checks the
whole configuration first (``-EINVAL``, or ``-E2BIG`` when the commands do not
fit in the SRAM), then writes it. Global fields:

=============== ===============================================================
Field           Description
=============== ===============================================================
``hostmode``    Host switch (``SW_HP`` pin): ``NPZERO_HOSTMODE_SWITCH``,
                ``_LOGIC`` (high when on) or ``_LOGICINV``
``pswint``      Bit ``n-1``: ``SW_LPn`` on in Standby state
``gateboost``   Gate boost of the power switches (lower Rds(on))
``wakeup``      Wake-up sources: ``NPZERO_SYSCFG1_WUP_P(n)``,
                ``NPZERO_SYSCFG1_WUP_A1`` (VBAT), ``NPZERO_SYSCFG1_WUP_A2``
                (``ADC_IN``)
``wakeupall``   Wake up when all the sources trigger, instead of any
``sclksel``     System clock: ``NPZERO_SCLK_LPOSC`` (10 Hz) or
                ``NPZERO_SCLK_XO`` (32.768 kHz crystal / 2048 = 16 Hz)
``sclkdiv``     System clock divider, ``NPZERO_SCLKDIV_1`` to ``_16``
``adcclk``      ADC clock, ``NPZERO_ADCCLK_SCLK`` (system clock) or
                ``_64HZ``, ``_256HZ``, ``_1024HZ`` (need ``NPZERO_SCLK_XO``)
``adcext``      Sample ``ADC_IN`` too (halves the sampling rate)
``iostrength``  High strength of the I/O pins
``i2cpull``     I2C pull-ups: ``NPZERO_I2CPULL_OFF``, ``_ON`` or ``_AUTO``
                (off in Idle state)
``spiauto``     SPI outputs in high impedance when not used
``clkout``      ``CLK_OUT`` pin, ``NPZERO_CLKOUT_OFF`` or 32 kHz to 1 kHz
``timeout``     Global time-out, in system clock periods (not 0)
``intpull[4]``  Pull-up of ``INTn``: ``NPZERO_INTPULL_OFF``, ``_100K`` or
                ``_50K``
``adcover[2]``, ADC thresholds of VBAT (``[0]``, 5-bit) and ``ADC_IN``
``adcunder[2]`` (``[1]``, 6-bit)
``periph[4]``   Peripherals 1 to 4, ``NULL`` when disabled
=============== ===============================================================

Fields of ``struct npzero_periph_s``:

=============== ===============================================================
Field           Description
=============== ===============================================================
``pwmod``       ``NPZERO_PWMOD_DISABLED``, ``_PERIODIC`` or ``_ALWAYSON``
``tmod``        Polling mode: ``NPZERO_TMOD_READ`` (init, read, compare),
                ``_INTREAD`` (init, wait for INT, read, compare), ``_INT``
                (init, wait for INT) or ``_ASYNCINT`` (wait for INT)
``pswmod``      ``SW_LP`` pin: ``NPZERO_PSWMOD_SWITCH``, ``_LOGIC`` or
                ``_LOGICINV``
``intmod``      ``INT`` pin: input (``NPZERO_INTMOD_IN_HIGH``/``_IN_LOW``) or
                trigger output (``_OUT_HIGH``/``_OUT_LOW``)
``spi``         SPI peripheral, otherwise I2C
``spimode``     SPI mode, 0 to 3
``dtype``       ``NPZERO_DTYPE_UINT16``, ``_INT16`` or ``_UINT8``
``inverted``    Trigger between the thresholds, instead of above or equal to
                ``throver`` or below or equal to ``thrunder``
``seqrw``       Multi-byte I2C transfers for sequential addresses
``wunak``       Wake up on an I2C NAK
``swap``        Swap the bytes of the value (big endian peripheral)
``addr``        7-bit I2C address of the peripheral
``rreg``        I2C register of the value (low byte)
``retries``     I2C retries on NAK, 0 to 3
``period``      Polling period, in system clock periods (2 or more)
``throver``,    Thresholds, in the type of ``dtype``
``thrunder``
``twtp``        Wait time count, in units of 256 or 4096 periods of the
                400 kHz oscillator (640 us or 10.24 ms)
``tinit``       Wait before the initialization: ``NPZERO_WAIT_NONE``,
                ``_256`` or ``_4096``
``twait``       Wait after the initialization, before the read or for the
                interrupt: ``NPZERO_WAIT_NONE``, ``_256`` or ``_4096``
``ninit``,      Initialization commands. I2C: ``ninit`` pairs of register
``init``        and value (``2 * ninit`` bytes). SPI: ``ninit`` bytes.
``nread``,      SPI only: bytes shifted out to read the value
``read``
=============== ===============================================================

The commands are stored in the SRAM in the order of the peripherals, the read
bytes of an SPI peripheral after its initialization bytes.

Example
=======

Configuration of the AS6212 temperature sensor of the DevKit on peripheral 4,
which wakes the host up above 25 C or below 10 C::

  static const uint8_t g_init[] =
  {
    0x01, 0x82, 0x02, 0xa0
  };

  static const struct npzero_periph_s g_temp =
  {
    .pwmod    = NPZERO_PWMOD_PERIODIC,
    .tmod     = NPZERO_TMOD_READ,
    .pswmod   = NPZERO_PSWMOD_LOGIC,
    .dtype    = NPZERO_DTYPE_INT16,
    .seqrw    = true,
    .swap     = true,
    .addr     = 0x49,
    .rreg     = 0x00,
    .period   = 300,                /* 30 s with the 10 Hz clock */
    .throver  = 25 * 128,
    .thrunder = 10 * 128,
    .twtp     = 49,
    .tinit    = NPZERO_WAIT_256,
    .twait    = NPZERO_WAIT_256,
    .ninit    = sizeof(g_init) / 2,
    .init     = g_init,
  };

  static const struct npzero_config_s g_config =
  {
    .hostmode = NPZERO_HOSTMODE_LOGIC,
    .pswint   = 1 << 3,
    .wakeup   = NPZERO_SYSCFG1_WUP_P(4),
    .sclkdiv  = NPZERO_SCLKDIV_1,
    .i2cpull  = NPZERO_I2CPULL_AUTO,
    .timeout  = 3000,               /* 300 s */
    .periph   =
    {
      NULL, NULL, NULL, &g_temp
    },
  };

  fd = open("/dev/npzero0", O_RDWR);
  ioctl(fd, NPZEROIOC_GETSTATUS, (unsigned long)&status);
  if (status.sta2 & NPZERO_STA2_FP(4))
    {
      value.periph = 4;
      ioctl(fd, NPZEROIOC_GETVALUE, (unsigned long)&value);
    }

  ioctl(fd, NPZEROIOC_CONFIGURE, (unsigned long)&g_config);
  ioctl(fd, NPZEROIOC_IDLE, 0);    /* The host is switched off */

The complete example is ``apps/examples/npzero``, see
:doc:`/applications/examples/npzero/index`, and the board is
:doc:`/platforms/arm/stm32l0/boards/npzero-g1s-dk/index`.

Differences with the Nanopower driver API
=========================================

The driver follows the datasheet NPZ-G1S-DS-V1.0 where the Nanopower driver
API does not:

- The host switch mode "power switch" is 0, the value 1 is invalid.
- The reset source is 4 bits (bit 3 is the soft reset) and the odd bits of
  STA2 are the NAK flags of the peripherals, not time-out flags.
- The ADC clocks derived from the crystal need ``NPZERO_SCLK_XO``.
- ``gateboost`` is written, and no ADC channel configuration is needed.
- The SRAM is written once, in 128 single writes, instead of 128 writes for
  each byte.
- ``tinit`` and ``twait`` are named after the ``TINIT`` and ``TWT`` fields of
  TCFGP. ``pre_wait_time`` of the Nanopower API sets ``TWT``.

=====================================
Switched-Mode Power Supply (``smps``)
=====================================

The SMPS driver is split into an "upper half" character driver
(``drivers/power/supply/smps.c``) and chip-specific "lower half" drivers.
It is enabled with ``CONFIG_DRIVERS_SMPS``. The application interface is
defined in ``include/nuttx/power/smps.h`` and
``include/nuttx/power/power_ioctl.h``. A lower half registers itself with
``smps_register()`` as ``/dev/smpsN``.

Application interface
=====================

========================= ============================ ===================
IOCTL                     Argument                     Description
========================= ============================ ===================
``PWRIOC_SET_MODE``       ``enum smps_opmode_e``       Select the
                                                       operation mode
``PWRIOC_SET_LIMITS``     ``struct smps_limits_s *``   Set the absolute
                                                       limits
``PWRIOC_SET_PARAMS``     ``struct smps_params_s *``   Set the output
                                                       voltage, current
                                                       and power
``PWRIOC_START``          none                         Start the
                                                       converter
``PWRIOC_STOP``           none                         Stop the converter
``PWRIOC_GET_STATE``      ``struct smps_state_s *``    Get the state and
                                                       the faults
``PWRIOC_GET_FAULT``      ``uint8_t *``                Get the faults
``PWRIOC_SET_FAULT``      ``uint8_t``                  Set a fault
``PWRIOC_CLEAN_FAULT``    ``uint8_t``                  Clear the latched
                                                       faults
========================= ============================ ===================

Commands that the upper half does not know are forwarded to the ``ioctl``
operation of the lower half.

Sequence
========

#. Select a mode with ``PWRIOC_SET_MODE``. The upper half refuses
   ``PWRIOC_START`` while the mode is ``SMPS_OPMODE_INIT``.
#. Set the limits with ``PWRIOC_SET_LIMITS``, with ``lock`` set to true. The
   limits can not be changed after they are locked, and the upper half
   refuses ``PWRIOC_START`` and ``PWRIOC_SET_PARAMS`` until they are set and
   locked.
#. Set the parameters with ``PWRIOC_SET_PARAMS``. The upper half refuses
   parameters above the limits with ``-EPERM``. The parameter that the mode
   regulates must be set: ``v_out`` for ``SMPS_OPMODE_CV``, ``i_out`` for
   ``SMPS_OPMODE_CC`` and ``p_out`` for ``SMPS_OPMODE_CP``. The parameters
   can be set again while the converter runs, unless they are locked.
#. Start and stop the converter with ``PWRIOC_START`` and ``PWRIOC_STOP``.

The lower half ``setup`` operation is called when the device is opened for
the first time and ``shutdown`` when it is closed for the last time, with
interrupts disabled. The open count is kept for every open, so closing a
descriptor does not shut the converter down while another one is open.

Supported chips
===============

========== ===== ====================================================
Chip       Bus   Notes
========== ===== ====================================================
LM51772    I2C   Buck-boost controller, constant voltage only. See
                 :doc:`lm51772`.
========== ===== ====================================================

The ``b-g474e-dpow1`` and ``stm32f334-disco`` boards register an SMPS
device that is driven by the :doc:`../../../../applications/examples/smps/index`
example.

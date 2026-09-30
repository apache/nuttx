=============
DShot Drivers
=============

DShot (Digital Shot) is a digital communication protocol used primarily for controlling brushless motor Electronic Speed Controllers (ESCs) in applications such as drones, multirotors, and other robotics systems. Unlike traditional PWM-based protocols, DShot provides digital communication that is more robust and faster, with optional bidirectional telemetry support.

The NuttX DShot driver is split into two parts:

#. An "upper half", generic driver that provides the common DShot interface to application level code, handles packet encoding/decoding, telemetry parsing, and manages the character device interface.
#. A "lower half", platform-specific driver that implements the low-level hardware controls to generate the DShot signal timing and optionally capture telemetry responses.

Files supporting DShot can be found in the following locations:

- **Interface Definition**: The header file for the NuttX DShot driver resides at ``include/nuttx/timers/dshot.h``. This header file includes both the application level interface to the DShot driver as well as the interface between the "upper half" and "lower half" drivers.
- **"Upper Half" Driver**: The generic, "upper half" DShot driver resides at ``drivers/timers/dshot.c``.
- **"Lower Half" Drivers**: Platform-specific DShot drivers reside in ``arch/<architecture>/src/<hardware>`` directory for the specific processor ``<architecture>`` and for the specific ``<chip>`` peripheral devices. For example, the i.MX9 FlexIO-based implementation is at ``arch/arm64/src/imx9/imx9_flexio_dshot.c``.

DShot Protocol Overview
=======================

The DShot protocol transmits 16-bit frames at fixed bit rates. Each frame consists of:

- **11 bits**: Throttle value (0-2047) or special command
- **1 bit**: Telemetry request flag
- **4 bits**: CRC checksum

Supported bit rates (speeds):

- **DShot150**: 150 kbit/s
- **DShot300**: 300 kbit/s
- **DShot600**: 600 kbit/s (most common)
- **DShot1200**: 1200 kbit/s
- **DShot2400**: 2400 kbit/s (experimental)
- **DShot3600**: 3600 kbit/s (experimental)

Bidirectional DShot
-------------------

Bidirectional DShot allows ESCs to send telemetry data back to the controller on the same signal line used for commands. The telemetry includes:

- **eRPM**: Electrical RPM of the motor
- **Extended telemetry**: Temperature, voltage, current, debug values, etc.

When bidirectional mode is enabled, the CRC is inverted in the command packet, and the ESC responds with a GCR-encoded telemetry frame after the command.

Application Level Interface
============================

To use the DShot driver in an application, include the header file:

.. code-block:: c

   #include <nuttx/timers/dshot.h>

The DShot driver is registered as a POSIX character device file into the ``/dev`` namespace. Open the device to get a file descriptor for operations:

.. code-block:: c

   int fd = open("/dev/dshot0", O_RDWR);
   if (fd < 0)
     {
       /* Handle error */
     }

Configuration
-------------

Before sending commands, configure the DShot instance with the desired speed, telemetry frequency, active channels, and bidirectional mode:

.. code-block:: c

   struct dshot_config_s config;

   config.freq = DSHOT_SPEED_600;      /* 600 kbit/s */
   config.telem_freq = 750000;         /* 1.25 * freq for standard ESCs */
   config.active_mask = 0x0F;          /* Enable channels 0-3 */
   config.bidir = true;                /* Enable bidirectional telemetry */

   int ret = ioctl(fd, DSHOTIOC_CONFIGURE, (unsigned long)&config);
   if (ret < 0)
     {
       /* Handle error */
     }

The ``telem_freq`` should typically be set to 1.25 times the bit rate frequency. However, some ESCs (e.g., T-motor F55A) may require different values like 1.15 times the bit rate.

Setting Throttle
----------------

Send throttle values or special commands to one or more channels:

.. code-block:: c

   struct dshot_throttle_s throttle;

   memset(&throttle, 0, sizeof(throttle));

   /* Set throttle values for channels 0-3 */
   throttle.throttle[0] = 1000;  /* Throttle range: 48-2047 (armed) */
   throttle.throttle[1] = 1000;  /*                 0 = disarm */
   throttle.throttle[2] = 1000;  /*                 1-47 = special commands */
   throttle.throttle[3] = 1000;
   throttle.ch_mask = 0x0F;      /* Update channels 0-3 */
   throttle.telemetry_req = 0x01; /* Request telemetry from channel 0 */

   ret = ioctl(fd, DSHOTIOC_SET_THROTTLE, (unsigned long)&throttle);
   if (ret < 0)
     {
       /* Handle error */
     }

Special Commands
----------------

DShot supports special commands for ESC control and configuration. These commands use throttle values 0-47:

.. code-block:: c

   struct dshot_throttle_s cmd;

   memset(&cmd, 0, sizeof(cmd));

   /* Send beep command to channel 0 */
   cmd.throttle[0] = DSHOT_CMD_BEEP1;
   cmd.ch_mask = 0x01;

   /* Commands should be sent multiple times (typically 10x) */
   for (int i = 0; i < 10; i++)
     {
       ioctl(fd, DSHOTIOC_SET_THROTTLE, (unsigned long)&cmd);
       usleep(1000); /* Small delay between commands */
     }

Common special commands:

- ``DSHOT_CMD_MOTOR_STOP`` (0): Disarm motor
- ``DSHOT_CMD_BEEP1`` through ``DSHOT_CMD_BEEP5`` (1-5): Audio tones
- ``DSHOT_CMD_ESC_INFO`` (6): Request ESC information
- ``DSHOT_CMD_SPIN_DIRECTION_NORMAL`` (20): Normal rotation
- ``DSHOT_CMD_SPIN_DIRECTION_REVERSED`` (21): Reversed rotation
- ``DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE`` (33): Enable telemetry
- ``DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY`` (34): Continuous eRPM

Reading Telemetry
-----------------

When bidirectional mode is enabled, retrieve telemetry data from ESCs:

.. code-block:: c

   struct dshot_telemetry_s telem;

   telem.ch_mask = 0x0F; /* Read telemetry from channels 0-3 */

   ret = ioctl(fd, DSHOTIOC_GET_TELEMETRY, (unsigned long)&telem);
   if (ret < 0)
     {
       /* Handle error */
     }

   /* Process telemetry for each channel */
   for (int i = 0; i < 4; i++)
     {
       printf("Channel %d:\n", i);
       printf("  eRPM: %u\n", telem.ch_telemetry[i].erpm);
       printf("  EDT Type: 0x%02x\n", telem.ch_telemetry[i].edt_type);
       printf("  EDT Value: %u\n", telem.ch_telemetry[i].edt_value);
     }

The telemetry structure also includes timestamps indicating when the last valid response was received from each ESC.

Efficient Telemetry Handling
-----------------------------

For efficient operation, the ``DSHOTIOC_SET_THROTTLE`` ioctl can both send commands and retrieve the previous telemetry response in a single call:

.. code-block:: c

   struct dshot_throttle_s throttle;

   memset(&throttle, 0, sizeof(throttle));
   throttle.throttle[0] = 1000;
   throttle.ch_mask = 0x01;
   throttle.telemetry_req = 0x01; /* Request telemetry */

   ret = ioctl(fd, DSHOTIOC_SET_THROTTLE, (unsigned long)&throttle);

   /* Previous telemetry is now available in throttle.ch_telemetry[] */
   printf("Previous eRPM: %u\n", throttle.ch_telemetry[0].erpm);

Configuration Options
=====================

The following configuration options control DShot driver behavior:

- ``CONFIG_DSHOT``: Enable DShot driver support
- ``CONFIG_DSHOT_NCHANNELS``: Maximum number of DShot channels (default: 8, max: 16)

.. note::

   In addition to these generic DShot options, you must enable and configure
   the architecture-specific lower-half driver. For example, on i.MX9:

   - ``CONFIG_IMX9_FLEXIO_DSHOT``: Enable FlexIO-based DShot driver
   - ``CONFIG_IMX9_FLEXIO_DSHOT_CHANNEL_COUNT``: Number of hardware channels

   Refer to your platform's Kconfig options for specific lower-half settings.

Example configuration in ``defconfig``:

.. code-block:: text

   CONFIG_DSHOT=y
   CONFIG_DSHOT_NCHANNELS=8

   # Platform-specific (example for i.MX9)
   CONFIG_IMX9_FLEXIO_DSHOT=y
   CONFIG_IMX9_FLEXIO_DSHOT_CHANNEL_COUNT=8

Lower Half Driver Interface
============================

The lower half driver must implement the operations defined in ``struct dshot_ops_s``:

.. code-block:: c

   struct dshot_ops_s
   {
     CODE int (*setup)(FAR struct dshot_lowerhalf_s *dev);
     CODE int (*shutdown)(FAR struct dshot_lowerhalf_s *dev);
     CODE int (*configure)(FAR struct dshot_lowerhalf_s *dev,
                           FAR const struct dshot_config_s *cfg);
     CODE int (*send_command)(FAR struct dshot_lowerhalf_s *dev,
                              FAR const uint16_t *packets,
                              uint16_t ch_mask);
     CODE int (*get_raw_telemetry)(FAR struct dshot_lowerhalf_s *dev,
                                   FAR struct dshot_raw_telemetry_s *raw,
                                   uint16_t ch_mask);
     CODE int (*ioctl)(FAR struct dshot_lowerhalf_s *dev,
                       int cmd, unsigned long arg);
   };

Operation Descriptions
----------------------

**setup**
  Called when the device is opened. Initialize hardware and allocate resources.

**shutdown**
  Called when the device is closed. Stop output and free hardware resources.

**configure**
  Configure the DShot speed (bit rate), telemetry capture frequency, active channel mask, and bidirectional mode. The lower half driver should reconfigure timers/peripherals accordingly.

**send_command**
  Transmit DShot packets to the specified channels. The ``packets`` array contains pre-encoded 16-bit DShot frames (including CRC), and ``ch_mask`` indicates which channels to update.

**get_raw_telemetry**
  Retrieve raw GCR-encoded telemetry data captured from the ESC responses. The upper half will decode these into human-readable values.

**ioctl** (optional)
  Handle platform-specific ioctl commands not covered by the standard interface.

Registering a Lower Half Driver
--------------------------------

The lower half driver registers itself with the upper half using:

.. code-block:: c

   int dshot_register(FAR const char *path, FAR struct dshot_lowerhalf_s *dev);

Example from board initialization code:

.. code-block:: c

   /* Initialize platform-specific DShot hardware */
   FAR struct dshot_lowerhalf_s *dshot = imx9_dshot_initialize();
   if (dshot == NULL)
     {
       return -ENODEV;
     }

   /* Register with upper half */
   ret = dshot_register("/dev/dshot0", dshot);
   if (ret < 0)
     {
       return ret;
     }

Implementation Considerations
==============================

Timing Requirements
-------------------

DShot requires precise timing. For DShot600:

- Bit period: 1.67 μs
- Logic '1': ~1.25 μs high, ~0.42 μs low (75% duty cycle)
- Logic '0': ~0.63 μs high, ~1.04 μs low (37.5% duty cycle)

Lower half implementations typically use hardware timers (PWM/DMA) or specialized peripherals like FlexIO to generate accurate waveforms.

Telemetry Capture
-----------------

Bidirectional telemetry requires the signal line to transition from output to input mode after sending the command. The ESC responds with a 21-bit GCR-encoded frame approximately 30-40 μs after the command. Accurate capture requires:

- Fast pin direction switching
- Precise edge timing measurement
- GCR decoding and CRC validation (handled by upper half)

Multi-Channel Support
---------------------

For multi-channel systems, consider:

- Synchronizing output across channels (important for motor control)
- Efficient DMA usage to minimize CPU overhead
- Buffer management for simultaneous telemetry capture

Best Practices
==============

1. **Initialization Sequence**

   - Wait for ESC to initialize (typically 3-5 seconds after power-on)
   - Configure DShot before attempting to send commands
   - Send ``DSHOT_CMD_MOTOR_STOP`` after initialization to ensure motors are disarmed

2. **Command Repetition**

   Special commands should be sent multiple times (typically 6-10 repetitions) to ensure reliable reception.

3. **Telemetry Polling**

   **Bidirectional Mode**: Requesting telemetry on every command cycle is normal and expected. The telemetry response is received on the same signal line immediately after the command.

   **Non-Bidirectional Mode** (dedicated UART telemetry): Consider the UART line capability and timing of response packets. Handle UART receive separately from DShot command transmission.

4. **Error Handling**

   Monitor telemetry timestamps to detect ESC communication failures. Stale telemetry may indicate signal integrity issues.

5. **Safety**

   Always send ``DSHOT_CMD_MOTOR_STOP`` (throttle value 0) before closing the device or in emergency situations.

Example Application
===================

Complete example of DShot motor control:

.. code-block:: c

   #include <nuttx/config.h>

   #include <fcntl.h>
   #include <stdio.h>
   #include <string.h>
   #include <unistd.h>

   #include <nuttx/timers/dshot.h>

   /****************************************************************************
    * Public Functions
    ****************************************************************************/

   int main(int argc, FAR char *argv[])
   {
     struct dshot_config_s config;
     struct dshot_throttle_s throttle;
     int fd;
     int ret;
     int i;

     /* Open DShot device */

     fd = open("/dev/dshot0", O_RDWR);
     if (fd < 0)
       {
         fprintf(stderr, "Failed to open DShot device\n");
         return -1;
       }

     /* Configure DShot600 with bidirectional telemetry */

     config.freq        = DSHOT_SPEED_600;
     config.telem_freq  = 750000;
     config.active_mask = 0x0f;  /* 4 motors */
     config.bidir       = true;

     ret = ioctl(fd, DSHOTIOC_CONFIGURE, (unsigned long)&config);
     if (ret < 0)
       {
         fprintf(stderr, "Configuration failed\n");
         close(fd);
         return -1;
       }

     /* Arm ESCs - send zero throttle */

     memset(&throttle, 0, sizeof(throttle));
     throttle.ch_mask = 0x0f;

     for (i = 0; i < 100; i++)
       {
         ioctl(fd, DSHOTIOC_SET_THROTTLE, (unsigned long)&throttle);
         usleep(10000);
       }

     /* Gradual throttle increase */

     for (i = 48; i <= 1000; i += 10)
       {
         throttle.throttle[0]   = i;
         throttle.throttle[1]   = i;
         throttle.throttle[2]   = i;
         throttle.throttle[3]   = i;
         throttle.telemetry_req = 0x01;

         ret = ioctl(fd, DSHOTIOC_SET_THROTTLE, (unsigned long)&throttle);
         if (ret == 0 && throttle.ch_telemetry[0].erpm > 0)
           {
             printf("Throttle: %d, eRPM: %u\n", i,
                    throttle.ch_telemetry[0].erpm);
           }

         usleep(20000);
       }

     /* Stop motors */

     memset(&throttle, 0, sizeof(throttle));
     throttle.ch_mask = 0x0f;
     ioctl(fd, DSHOTIOC_SET_THROTTLE, (unsigned long)&throttle);

     close(fd);
     return 0;
   }

References
==========

- DShot Protocol Specification: https://github.com/betaflight/betaflight/wiki/DShot
- ESC Telemetry Protocol: https://github.com/bird-sanctuary/extended-dshot-telemetry
- DShot and Bidirectional DShot: https://brushlesswhoop.com/dshot-and-bidirectional-dshot/

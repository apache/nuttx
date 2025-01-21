=======================
Goldfish Sensor Driver
=======================

Introduction
============

The Goldfish Sensor driver provides a virtual multi-sensor interface for
Android Goldfish/QEMU emulator environments. This driver enables NuttX to
interact with the QEMU sensor emulation infrastructure, allowing applications
to access various simulated sensor types without requiring physical hardware.

The driver supports multiple sensor types through a unified interface based
on the NuttX sensor framework (uORB), making it suitable for development,
testing, and debugging of sensor-dependent applications in virtualized
environments.

Supported Sensor Types
=======================

The Goldfish Sensor driver supports the following sensor types:

Inertial Sensors
----------------

- **Accelerometer** (``SENSOR_TYPE_ACCELEROMETER``)

  - Measures linear acceleration in three axes (x, y, z)
  - Range: ±2.8 m/s²
  - Resolution: 1.0/4032.0 m/s²
  - Power consumption: 3.0 mW

- **Gyroscope** (``SENSOR_TYPE_GYROSCOPE``)

  - Measures angular velocity in three axes
  - Range: ±11.11 rad/s
  - Resolution: 1.0/1000.0 rad/s
  - Power consumption: 3.0 mW

- **Uncalibrated Accelerometer** (``SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED``)

  - Raw accelerometer data without calibration
  - Range: ±39.3 m/s²
  - Resolution: 1.0/4032.0 m/s²

- **Uncalibrated Gyroscope** (``SENSOR_TYPE_GYROSCOPE_UNCALIBRATED``)

  - Raw gyroscope data without drift compensation
  - Range: ±16.46 rad/s
  - Resolution: 1.0/1000.0 rad/s

Magnetic Sensors
----------------

- **Magnetometer** (``SENSOR_TYPE_MAGNETIC_FIELD``)

  - Measures magnetic field strength in three axes
  - Range: ±2000.0 μT
  - Resolution: 1.0 μT
  - Power consumption: 6.7 mW

- **Uncalibrated Magnetometer** (``SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED``)

  - Raw magnetic field data without hard/soft iron correction
  - Range: ±2000.0 μT
  - Resolution: 1.0 μT

Environmental Sensors
---------------------

- **Ambient Temperature** (``SENSOR_TYPE_AMBIENT_TEMPERATURE``)

  - Measures ambient temperature
  - Range: 0-80°C
  - Resolution: 1.0°C
  - Power consumption: 0.0 mW

- **Barometric Pressure** (``SENSOR_TYPE_BAROMETER``)

  - Measures atmospheric pressure
  - Range: 0-800 hPa
  - Resolution: 1.0 hPa
  - Power consumption: 20.0 mW

- **Relative Humidity** (``SENSOR_TYPE_RELATIVE_HUMIDITY``)

  - Measures relative humidity
  - Range: 0-100%
  - Resolution: 1.0%
  - Power consumption: 20.0 mW

Optical Sensors
---------------

- **Ambient Light** (``SENSOR_TYPE_LIGHT``)

  - Measures ambient light intensity
  - Range: 0-40000 lux
  - Resolution: 1.0 lux
  - Power consumption: 20.0 mW

- **Proximity** (``SENSOR_TYPE_PROXIMITY``)

  - Detects nearby objects
  - Range: 0-1.0 (binary detection)
  - Resolution: 1.0
  - Power consumption: 20.0 mW

Orientation Sensors
-------------------

- **Orientation** (``SENSOR_TYPE_ORIENTATION``)

  - Provides device orientation (azimuth, pitch, roll)
  - Range: 0-360°
  - Resolution: 1.0°
  - Power consumption: 9.7 mW

Position Sensors
----------------

- **Hinge Angle 0/1/2** (``SENSOR_TYPE_HINGE_ANGLE``)

  - Measures foldable device hinge angles
  - Range: 0-360°
  - Resolution: 1.0°
  - Power consumption: 3.0 mW
  - Note: Supports up to 3 hinges (for multi-fold devices)

Biometric Sensors
-----------------

- **Heart Rate** (``SENSOR_TYPE_HEART_RATE``)

  - Measures heart rate in beats per minute
  - Range: 0-500 bpm
  - Resolution: 1.0 bpm
  - Power consumption: 20.0 mW

Gesture Sensors
---------------

- **Wrist Tilt** (``SENSOR_TYPE_WRIST_TILT_GESTURE``)

  - Detects wrist tilt gesture events
  - Range: 0-1.0 (event trigger)
  - Resolution: 1.0
  - Power consumption: 20.0 mW

Usage
=====

Kernel Configuration
--------------------

Enable the Goldfish Sensor driver in your NuttX configuration::

    CONFIG_SENSORS=y
    CONFIG_SENSORS_GOLDFISH=y
    CONFIG_GOLDFISH_PIPE=y

Driver Initialization
---------------------

The driver is typically initialized during board-level setup::

    #include <nuttx/sensors/goldfish_sensor.h>

    int board_sensors_initialize(void)
    {
      int ret;

      /* Initialize Goldfish sensors with device number 0
       * and batch buffer size of 1
       */
      ret = goldfish_sensor_init(0, 1);
      if (ret < 0)
        {
          snerr("ERROR: Failed to initialize Goldfish sensors: %d\n", ret);
          return ret;
        }

      return OK;
    }

Application Usage
-----------------

Applications can access sensors through the standard NuttX sensor interface::

    #include <fcntl.h>
    #include <nuttx/sensors/sensor.h>

    int main(int argc, char *argv[])
    {
      struct sensor_accel accel_data;
      int fd;
      int ret;

      /* Open accelerometer sensor */
      fd = open("/dev/uorb/sensor_accel0", O_RDONLY);
      if (fd < 0)
        {
          printf("Failed to open accelerometer\n");
          return -1;
        }

      /* Read sensor data */
      while (1)
        {
          ret = read(fd, &accel_data, sizeof(accel_data));
          if (ret == sizeof(accel_data))
            {
              printf("Accel: x=%.2f y=%.2f z=%.2f (timestamp=%llu)\n",
                     accel_data.x, accel_data.y, accel_data.z,
                     accel_data.timestamp);
            }

          usleep(100000); /* 100ms */
        }

      close(fd);
      return 0;
    }

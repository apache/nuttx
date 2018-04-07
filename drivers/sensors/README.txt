ADXL345 (Alan Carvalho de Assis)
=======

The ADXL345 accelerometer can operate in I2C or SPI mode. To operate in I2C
mode just connect the CS pin to Vddi/o.

In order to operate in SPI mode CS need to use connected to microcontroller,
it cannot leave unconnected.

In SPI mode it works with clock polarity (CPOL) = 1 and clock phase (CPHA)
= 1.

ADXL372 (Bob Feretich)
=======

The ADXL372 is a 200g tri-axis accelerometer that is capable of detecting
and recording shock impact impact events. Recording trigger
characteristics are programed into the sensor via multiple threshold and
duration registers.  The ADXL372 is a SPI only device that can transfer
data at 10 MHz.  The data transfer performance of this part permits the
sensor to be sampled "on demand" rather than periodically sampled by a
worker task.

See the description of the "Common Sensor Register Interface" below for more
details. It also implements the "Sensor Cluster Driver Interface".

LSM330_SPI (Bob Feretich)
==========

The LSM330 consists of a multi-range tri-axis accelerometer and a
multi-range tri-axis gyroscope. The tri-axis accelerometer features two
state machines that can be firmware programmed for event detection. The
tri-axis gyroscope features threshold and duration registers for event
detection.

This driver supports the LSM330 in SPI mode. In this mode, the LSM330
that can transfer data at 10 MHz. The data transfer performance of
this part permits the sensor to be sampled "on demand" rather than
periodically sampled by a worker task. See the description of the "Common
Sensor Register Interface" below for more details. It also implements the
"Sensor Cluster Driver Interface".

MPL115A (Alan Carvalho de Assis)
=======

This driver has support only for MPL115A1 (SPI), but support to MPL115A2
(I2C) can be added easily.

Common Sensor Register Interface (Bob Feretich)
================================

Background and problem statement:

The capabilities and performance of modern sensors have grown tremendously.
Most sensors are now capable of some degree of autonomous behavior and
several permit the user to load firmware into them and perform as
nanocontrollers.  Other sensors have very sophisticated built-in digital
filters that can be programmed with hundreds of parameters.

Currently most sensor drivers in the Nuttx drivers/sensors
directory implement file_ops open(), close(), and read() functions.
The open() function initializes the sensor and places it in a mode where
it can transfer live data in a default configuration. The close() function
places the sensor in a low power shutdown mode. The read() function
returns the most recent data sample from the sensor's most used data
output registers.  The write() function is rarely implemented and when it
is there is no consistency in its use. The lseek() and poll() functions
seem to be completely ignored.  This results in the sensors being operated
in only their most primitive modes using a fixed "default configuration".

To work around this problem sensor drivers have implemented ioctl()
functions to perform configuration, program the sensor, and manage
autonomous activity.  Ioctls provide a method where user programs can
tunnel through a high level driver to access and control device specific
features. The problem with using ioctls is that before the ioctl interface
can be used, the sensor driver must be opened; and the open() function
causes the driver to start performing these primitive actions, so before
ioctls can manage the drivers as desired, ioctls must first be used to
undo the generic actions caused by the open() function. Another major
issue is that there is no consistency from sensor to sensor on ioctl
definitions, not even for the most common sensor actions like writing a
sensor control register or reading a sensor status register.

Purpose:

The purpose of the "Common Sensor Register Interface" is to implement a
consistent and more useful definition of file_ops interface and to make the
file_ops open() function more flexible in establishing the initial
operational state of the sensor. Compatibility for user applications that
implement the current open(), close(), read() interface will be
maintained; and the much greater capabilities of modern sensors will
become accessible through this interface.

Scope:

Applicable to I2C and SPI attached sensors, and some serial port attached
sensors.

The file_ops interface definition:

open(): This function performs the below actions...

  1) Reads the sensors ID register. If the sensor responds with an
     unexpected value, then...

     a) The driver's write() function is disabled.
     b) The open function initializes the driver instance, so
        that read() and lseek() operations may be performed to enable
        problem diagnoses, but the sensor hardware is not initialized.
        (No write operations are performed to the sensor.)
     c) The errno global variable is set to positive ENODEV
        ("No such device").
     d) The open() function returns successfully with a file_handle.
         Note that the calling routine should clear errno before
         calling open(). (The file_ops rules prevent drivers from
         setting errno to zero.)

  2) The other file_ops functions are enabled.
  3) The driver's "current reg address" state variable is set to the
     sensor's first sensor data output register. (This will make
     calls to read() return live sensor data and maintain compatibility
     with existing user programs.)
  4) If the driver supports a default worker task and an interrupt
     handler is specified by in the sensor configuration structure, then
     the default worker task is bound to the default worker task.
  5) The sensor configuration structure (that was provided to the driver
     registration function) is examined to determine whether a custom
     sensor configuration is specified. (The custom configuration is
     basically an array of (device_reg_address, value) pairs that are
     written to the sensor via "single register write" operations.
     If a custom sensor configuration was specified, then the that
     configuration is written to the sensor, otherwise the "default
     sensor configuration" is written to the sensor.
     (A side effect of writing this data may result in interrupts
     occurring and data being transferred to/from the worker task.)
  6) The open() function returns successfully with a file_handle.

close(): This function stops sensor activity and places it in a low
  power mode. The file_ops interface functions are disabled for this
  instance of the sensor driver. (Except for open())

read(): The action of this function is dependent on whether a "default
  worker task" is running and the value of the driver's "current reg
  address" state variable.

  If a "default worker task" is running,

    AND the driver's "current reg address" is equal to the value of
      the first sensor data output register,
    AND the number of bytes to be read is less than or equal to the
      number of bytes in a "default worker task" sample,

  Then data is copied from the "default worker task's" sample memory to
  the caller's provided buffer.

  Otherwise, this function transfers data from sensor registers to the
  data buffer provided by the caller. The first byte read is from the
  sensor register address specified by the sensor's "current reg
  address". The addresses of subsequent bytes to be read are context
  sensitive. If more than bus transfer is needed to complete the read,
  then a "multi-byte" (sometimes called "burst mode") data transfer
  will be used to fill the buffer.
  See the sensor's datasheet to determine the auto-increment
  behavior of a "multi-byte" data transfers.

  Note: That most sensors collect only a few bytes of data per sample.
  Small data transfers occurring over a high speed bus (like SPI and some
  high speed i2c and serial interfaces) are much more efficient when
  collected directly from the sensor hardware than by using a worker task
  as an intermediary.

write():  This function transfers data from the data buffer provided by
  the caller to sensor registers.  The first byte written is to the
  sensor register address specified by the sensor's "current reg
  address".  The addresses of subsequent bytes to be read are context
  sensitive.  If more than bus transfer is needed to complete the write,
  then a "multi-byte" (sometimes called "burst mode") data
  transfer will be used to transfer data from the buffer.

  See the sensor's datasheet to determine the auto-increment
  behavior of a "multi-byte" data transfers.

  Note: If write() function was disabled, then no writes will be performed
  and the function will return 0 (characters transferred) and errno
  is set to -EROFS ("read-only file system").

lseek(): This function sets the value of the sensor's "current reg address"
  (seek_address). The open() function initializes the "current reg address"
  to the first sensor data output register, so unless the user needs
  to change the sensor configuration, lseek() does not need to be
  called. Neither read() nor write() change the sensor's "current reg
  address".

  The definition of lseek is...

      off_t lseek(int fd, off_t offset, int whence);

  For whence == SEEK_SET, the sensor's "current reg address" will be set
  to offset.

  For whence == SEEK_CUR, offset will be added to the sensor's "current
  reg address".

  For whence == SEEK_END, offset is ignored and the sensor's "current
  reg address" is set to the first sensor data output register.

  lseek() will return an error if the resulting "current reg address"
  is invalid for the sensor.

ioctl(): Ioctls() may still be used and this interface make no attempt to
  regulate them. But, it is expected that far fewer ioctls will be needed.

The above interface can be used to fully configure a sensor to the needs
of an application, including the ability to load firmware into sensor
state machines

Sensor Cluster Driver Interface:(Bob Feretich)
===============================

Background and problem statement:

Most microcontrollers can support SPI bus transfers at 8 MHz or greater.
Most SPI attached sensors can support a 10 MHz SPI bus.  Most tri-axis
accelerometers, tri-axis gyroscopes, or tri-axis magnetometers use only 6
bytes per sample. Many sensors use less than 6 bytes per sample.  On an 8
MHz SPI bus it takes about 8 microseconds to transfer a 6 byte sample.
(This time includes a command byte, 6 data bytes, and chip select select
setup and hold.) So, for the below discussion keep in mind that the sensor
sample collection work we want to perform should ideally take 8 microseconds
per sample.

The drivers in the drivers/sensors directory support only the user space
file_ops interface (accessing drivers through the POSIX open/read/close
functions using a file descriptor). Also these drivers typically start
their own worker task to perform sensor data collection, even when their
sensors only transfer a few bytes of data per sample and those transfers
are being made over a high performance bus.

Using the current implementation...

 1) A sensor "data ready" or timer interrupt occurs.
 2) Context is saved and and the driver's interrupt handler is scheduled
    to run.
 3) The Nuttx scheduler dispatches the driver's interrupt handler task.
 4) The driver's interrupt handler task posts to a semaphore that the
    driver's worker task is waiting on.
 5) Nuttx restores the context for the driver's worker task and starts it
    running.
 6) The driver's worker task starts the i/o to collect the sample.) (This is
    where the 8 microseconds of real work gets performed.) And waits on a
    SPI data transfer complete semaphore.
 7) The Nuttx saves the context of the driver's worker task, and the
    scheduler dispatches some other task to run while we are waiting.
    Note that this is a good thing. This task is probably performing some
    other real work. We want this to happen during the data transfer.
 8) The completion of the data transfer causes an interrupt. Nuttx saves the
    current context and restores the driver's worker task's context.
 9) The driver's worker task goes to sleep waiting on the semaphore for the
    next sensor "data ready" or timer interrupt.
10) The Nuttx saves the context of the driver's worker task, and the
    scheduler dispatches some other task to run while we are waiting.

Independently with the above...

a) The sensor application program performs a file_ops read() to collect a
   sample.
b) The Nuttx high level driver receives control, performs a thin layer of
   housekeeping and calls the sensor driver's read function.
c) The sensor driver's read function copies the most recent sample from the
   worker task's data area to the application's buffer and returns.
d) The Nuttx high level driver receives control, performs a thin layer of
   housekeeping and returns.
e) The application processes the sample.

Using a 216 MHz STM32F7 with no other activity occurring, we have timed the
above the elapsed time for the above to be on average 45 microseconds.

Most sensor applications process data from multiple sensors. (An 9-DoF IMU
is typically represented as three sensors (accelerometer, gyroscope, and
magnetometer). In this case there are three copies of 1-10 occurring in
parallel.

In applications where live data is being used, the context switch
thrashing and cache pollution of this approach cripples system
performance.  In applications where sensor FIFO data is being used and
therefore a large amount of data is collected per iteration, the non "zero
copy" nature of the data collection becomes a performance issue.

Purpose:

The "Sensor Cluster Driver Interface" provides a standard mechanism for
an application to collect data from multiple sensor drivers in a much more
efficient manner. It significantly reduces the number of running tasks and
the context thrashing and cache pollution caused by them. It also permits
"zero copy" collection of sensor data.

The Sensor Cluster Driver Interface uses a single "worker task" to be shared
by an arbitrary number of drivers. This shared worker task is a kernel
task that is registered like a driver, supports a driver interface to
application programs, and collects data from multiple sensors (a cluster of
sensors), we refer to it a "Sensor Cluster Driver".

Its goal is to change the sequence of events detailed above to...

 1) A sensor "data ready" or timer interrupt occurs.
 2) Context is saved and and the cluster driver's interrupt handler is
    scheduled to run.
 3) The Nuttx scheduler dispatches the cluster driver's interrupt handler
    task.
 4) The cluster driver's interrupt handler task posts to a semaphore that
    the cluster driver's worker task is waiting on.
 5) Nuttx restores the context for the driver's worker task and starts it
    running.
 6) The cluster driver's worker task starts the i/o to collect the sample.
    There are two choices here. Programed I/O (PIO) or DMA. If PIO is
    fastest for a small sample size, but it will lock up the processor for
    the full duration of the transfer; it can only transfer from one
    sensor at a time; and the worker task should manually yield control
    occasionally to permit other tasks to run. DMA has higher start and
    completion overhead, but it is much faster for long transfers, can
    perform simultaneous transfers from sensors on different buses, and
    automatically releases the processor while the transfer is occurring.
    For this reason our drivers allows the worker task to choose between
    PIO (driver_read()) and DMA (driver_exchange()), a common extension to
    the sensor_cluster_operations_s structure. So either way after one or
    more transfers we yield control and move to the next step. Note that
    the data is being transferred directly into the buffer provided by the
    application program; so no copy needs to be performed.
 7) The Nuttx saves the context of the cluster driver's worker task, and the
    scheduler dispatches some other task to run while we are waiting.
    Again note that this is a good thing. This task is probably performing
    some other real work. We want this to happen during the data transfer.
 8) The completion of the last of the previous data transfers causes an
    interrupt.  Nuttx saves the current context and restores the cluster
    driver's worker task's context. If there is more sensor data to
    collect, then goto Step 6.  Otherwise it posts to a semaphore that
    will wake the application.
 9) The driver's worker task goes to sleep waiting on the semaphore for the
    next sensor "data ready" or timer interrupt.
10) The Nuttx saves the context of the driver's worker task, and the
    scheduler dispatches some other task to run while we are waiting.

Independently with the above...

a) The sensor application program performs a file_ops read() to collect a
   sample.
b) The Nuttx high level driver receives control, performs a thin layer of
   housekeeping and calls the sensor driver's read function.
c) The sensor driver's read function copies the most recent sample from the
   worker task's data area to the application's buffer and returns.
d) The Nuttx high level driver receives control, performs a thin layer of
   housekeeping and returns.
e) The application processes the sample.

So when collecting data from three sensors, this mechanism saved...

  * the handling of 2 sensor "data ready" or timer interrupts (Steps 1 - 4).
  * 2 occurrences of waking and scheduling of a worker task (Step 5).
  * 2 context switches to other tasks (Step 9 & 10)
  * if the three sensors were on separate buses, then 2 occurrences of

Steps 6 - 8 could have also been saved.

  * An extra copy operation of the collected sensor data.
  * The cache pollution caused by 2 competing worker tasks.

Definitions:

Leaf Driver - a kernel driver that implements the "Sensor Cluster Driver
  Interface" so that it can be called by Cluster drivers.
Cluster Driver - a kernel driver that uses the "Sensor Cluster Driver
  Interface" to call leaf drivers.
Entry-Point Vector - an array of function addresses to which a leaf driver
  will permit calls by a Cluster Driver.
Leaf Driver Instance Handle - a pointer to an opaque Leaf Driver structure
  that identifies an instance of the leaf driver. Leaf Drivers store this
  handle in its configuration structure during registration.

Sensor Cluster Interface description:

 * The definition of an entry-point vector. This is similar to the
   entry-point vector that is provided to the file-ops high level driver.
   This entry-point vector must include the sensor_cluster_operations_s
   structure as its first member.
 * The the definition of an driver entry-point vector member in the leaf
   driver's configuration structure. The leaf driver registration function
   must store the address of its entry-point vector in this field.
 * The the definition of an instance handle member in the leaf drivers
   configuration structure. The leaf driver registration function must store
   a handle (opaque pointer) to the instance of the leaf driver being
   registered in this field. Note that this should be the same handle that
   the leaf driver supplies to Nuttx to register itself. The cluster driver
   will include this handle as a parameter in calls made to the leaf driver.

struct sensor_cluster_operations_s
{
  CODE int     (*driver_open)(FAR void *instance_handle, int32_t arg);
  CODE int     (*driver_close)(FAR void *instance_handle, int32_t arg);
  CODE ssize_t (*driver_read)(FAR void *instance_handle, FAR char *buffer,
                              size_t buflen);
  CODE ssize_t (*driver_write)(FAR void *instance_handle,
                               FAR const char *buffer, size_t buflen);
  CODE off_t   (*driver_seek)(FAR void *instance_handle, off_t offset,
                              int whence);
  CODE int     (*driver_ioctl)(FAR void *instance_handle, int cmd,
                               unsigned long arg);
  CODE int     (*driver_suspend)(FAR void *instance_handle, int32_t arg);
  CODE int     (*driver_resume)(FAR void *instance_handle, int32_t arg);
};

Note that the sensor_cluster_operations_s strongly resembles the Nuttx fs.h
file_operations structures. This permits the current file_operations
functions to become thin wrappers around these functions.

driver_open(): Same as the fs.h open() except that arg can be specify
  permitting more flexibility in sensor configuration and initial operation.
  when arg = 0 the function of driver_open() must be identical to open().

driver_close(): Same as the fs.h close() except that arg can be specify
  permitting more flexibility in selecting a sensor low power state.
  when arg = 0 the function of driver_close() must be identical to close().

driver_read(): Same as the fs.h read().

driver_write(): Same as the fs.h write(). Optional. Set to NULL if not
  supported.

driver_seek(): Same as the fs.h seek(). Optional. Set to NULL if not
  supported.

driver_ioctl(): Same as the fs.h ioctl(). Optional. Set to NULL if not
  supported.

driver_suspend() and driver_resume(): Optional. Set to NULL if not
  supported.  It is common for sensor applications to conserve power and
  send their microcontroller into a low power sleep state. It seems
  appropriate to reserve these spots for future use. These driver entry
  points exist in Linux and Windows. Since microcontrollers and sensors
  get more capable every year, there should soon be a requirement for
  these entry points.  Discussion on how to standardize their use and
  implementation should
  be taken up independently from this driver document.

Note that all drivers are encouraged to extend their entry-point vectors
beyond this common segment. For example it may be beneficial for the
worker task  to select between programed i/o and DMA data transfer
routines. Unregulated extensions to the Entry-Point Vector should be
encouraged to maximize the benefits of a sensor's features.

Operation:

Board logic (configs directory) will register the cluster driver. The
cluster driver will register the leaf drivers that it will call.
This means that the cluster driver has access to the leaf driver's
configuration structures and can pass the Leaf Driver Instance Handle to
the leaf driver as a parameter in calls made via the Entry-Point Vector.

Either board logic or an application program may open() the cluster
driver. The cluster driver open() calls the open() function of the leaf
drivers.  The cluster driver open() or read() function can launch the
shared worker task that collects the data.

The cluster driver close() function calls the close functions of the leaf
drivers.

===========================
GNSS Lower Half uORB Driver
===========================

The GNSS lower half driver is used to create uORB drivers for GNSS/GPS devices.
The upper-half driver abstracts away the parsing and advertising of NMEA data
from the device. The way this lower-half is instantiated is similar to lower
half drivers in the :doc:`uORB framework
</components/drivers/special/sensors/sensors_uorb>`.

For an example on how to use this lower-half, see
`</components/drivers/special/sensors/l86xxx>`

Application Programming Interface
=================================

To use the GNSS lower half in your driver, just include the following header:

.. code-block:: c

      #include <nuttx/sensors/gnss.h>


The GNSS driver works similarly to the uORB lower-half driver, where you must
implement several operations for the upper-half to work. The first step is
defining a custom type for your GNSS device, which must include a reference to
the lower-half struct. In this case, the device is UART based, so several
members are included to facilitate its operation.

.. code-block:: c

   /* Custom GNSS device type */

   typedef struct
   {
     FAR struct file uart;          /* UART interface to get data */
     struct gnss_lowerhalf_s lower; /* GNSS lower-half */
     bool enabled;                  /* Enabled state */
     char buffer[256];              /* UART read buffer */
     mutex_t lock;                  /* Device lock */
     sem_t run;                     /* Start/stop kthread */
   } my_gnss_dev_s;


Then, you can create the operations table.

.. code-block:: c

   static const struct gnss_ops_s g_gnss_ops =
   {
     .control = my_gnss_control,
     .activate = my_gnss_activate,
     .set_interval = my_gnss_set_interval,
   }

These functions are very similar to the functions that you use for implementing
a uORB lower-half.

* The ``control`` function is used to handle ``IOCTL`` commands that aren't
  implemented by the upper-half

* The ``activate`` function is used to enable/disable the device so that is
  saves power when not in use

* The ``set_interval`` function is used to set the sampling rate

It is up to you to implement these functions so that they meet the requirements
of the upper-half driver.

For this implementation, because we are reading data from a UART interface, we
will read from the ``uart`` member of our ``my_gnss_dev_s`` struct into our read
buffer. This takes place in a kernel thread, which polls the UART interface.
Once the data is read, we just have to send it to the upper-half to parse and
publish as uORB data.

The data we provide to the upper-half does not have to be null-terminated
strings, or even a complete NMEA sentence. The upper-half will parse as it goes,
and wait until it has a full sentence in its own buffer before parsing.

Here is an excerpt of how to publish that data:

.. code-block:: c

   /* `dev` is a reference to our `my_gnss_dev_s` struct */

   err = nxmutex_lock(&dev->lock);
   if (err < 0)
     {
       snerr("Couldn't lock mutex\n");
       return err;
     }

   bw = file_read(&dev->uart, dev->buffer, sizeof(dev->buffer));

   if (bw <= 0)
     {
       snerr("No data on UART: %d\n", bw);
       nxmutex_unlock(&dev->lock);
       continue;
     }

   /* Send data read to the lower half for parsing. Does not need to be a
    * full NMEA sentence to be handled.
    */

   if (bw > 0)
     {
       dev->lower.push_data(dev->lower.priv, dev->buffer, bw, true);
     }

   nxmutex_unlock(&dev->lock);

Once all of the above has been implemented, in your driver's registration
function, you'll need to initialize the structure to your GNSS device in the
registration function.

.. code-block:: c

   /* Registration function for this device type */

   int mygnss_register(FAR char const *uartpath, int devno)
   {
      FAR mygnss_dev_s *priv;
      int err;
      uint32_t nbuffers[SENSOR_GNSS_IDX_GNSS_MAX];

      /* This is a bare example not considering error handling */

      priv = kmm_zalloc(sizeof(my_gnss_dev_s));

      /* Initialize whatever specific members of your device struct need
       * initializing here...
       */
      
      priv->lower.ops = &g_gnss_ops; /* Ops table */
      priv->lower.priv = priv; /* Reference to your lower-half */

      /* This selects the buffer sizes for each of the buffers that handle these
       * sets of events. The index macros are included in the gnss header file
       */

      nbuffers[SENSOR_GNSS_IDX_GNSS] = 1;
      nbuffers[SENSOR_GNSS_IDX_GNSS_SATELLITE] = 1;
      nbuffers[SENSOR_GNSS_IDX_GNSS_MEASUREMENT] = 1;
      nbuffers[SENSOR_GNSS_IDX_GNSS_CLOCK] = 1;
      nbuffers[SENSOR_GNSS_IDX_GNSS_GEOFENCE] = 1;

      /* Register the lower-half driver with our information.
       * `SENSOR_GNSS_IDX_GNSS_MAX` is the length of our `nbuffers` array.
       */

      err =
          gnss_register(&priv->lower, devno, nbuffers, SENSOR_GNSS_IDX_GNSS_MAX);
      if (err < 0)
        {
          snerr("Failed to register myGNSS driver: %d\n", err);
          /* You should handle the error by cleaning up resources */
        }

      /* Here, handle starting up your kernel thread and any other error
       * cleanup.
       */

      return err;
   }

Registration in Device Code
===========================

To register the driver in device code, simply call the registration function you
wrote. You'll have to include your header file.

.. code-block:: c

   #if defined(CONFIG_SENSORS_MYGNSS) /* Change for your GNSS driver */
   #include <nuttx/sensors/mygnss.h>
   #endif

   /* Put this inside your board's real bringup function, where other drivers
    * are being registered.
    */

   int my_board_bringup(void)
   {
     #if defined(CONFIG_SENSORS_MYGNSS)
       /* Register myGNSS on USART0 */
     
       ret = l86xxx_register("/dev/ttyS0", 0);
       if (ret < 0) {
         syslog(LOG_ERR, "Failed to register myGNSS driver: %d\n", ret);
       }
     #endif
   }

Operation
=========

Now you should see several different uORB GNSS topics get published under
``/dev/uorb``. These correspond to all of the different buffer types you
initialized earlier, like geofence, clock, statellite, etc. The plain
``sensor_gnss`` device will publish lots of data from the NMEA sentences.

You can use the ``uorb_listener`` application
(:doc:`/applications/system/uorb/index`) to see if data is being published
correctly.

.. code-block:: console

   nsh> uorb_listener sensor_gnss
   
   Monitor objects num:2
   object_name:sensor_gnss, object_instance:0
   sensor_gnss(now:237030000):timestamp:237030000,time_utc:1753502745,latitude:xxxxxxxx,longitude:xxxxxxxxxx,altitude:36.900002,altitude_ellipsoid:24.200001,0

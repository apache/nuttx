==========================================
``uorb`` uorb(micro object request broker)
==========================================

``uORB (Micro Object Request Broker)`` is a crucial middleware in the
open-source flight control system PX4. It is a pub-sub-based
message bus primarily responsible for data transmission between
multiple modules. Based on a lock-free design philosophy, uORB
internally implements Inter-Process Communication (IPC) among
tasks through shared memory and achieves low-latency data
exchange with an optimal memory footprint. Its implementation
does not rely on threads or work queues.

The main concepts related to uORB include ``roles``, and ``device nodes``.

**Roles**
^^^^^^^^^

In the uORB bus, there are two roles: subscribers and advertisers.
The transmitted content is referred to as a topic message, described
by its metadata (meta), including name, size, etc. 

Each topic supports multiple instances, with the advertiser frequency
being the sampling rate (interval) and the maximum publication delay
being the batch latency.

For example, if the accelerometer sensor has a sampling rate of 50Hz
and a maximum publication delay of 100ms, the hardware generates an
interrupt every 100ms, publishing 5 data points each time (100/(1000/50)).

**Two Tpyes**
^^^^^^^^^^^^^

Additionally, NuttX categorizes topics into two types: 
**notification** topics and **general** topics.

Subscribers to **notification** topics are not concerned with the presence
of an advertiser or whether a publication has occurred; they directly
obtain the current state.When an application subscribes, it directly
obtains the latest data from the current device node as the current state.

Subscribers to **general** topics are not concerned with past states;
the data they obtain must be current or future occurrences.
For example, with an accelerometer sensor, an application only cares
about the data reported by the next data ready interrupt from the sensor,
rather than data from a past moment.

To publish **notification** topic messages, APIs with the **persist** suffix
are used for topic publication, and subscriber behavior is consistent with
that of **general** topics.

**Device Nodes**
^^^^^^^^^^^^^^^^

Each topic being published or subscribed to corresponds to a character
device node, with subscribers and advertisers sharing data through an internal
circular buffer. In implementation, a caller who opens a device node in O_WRONLY
mode is considered an advertiser, and one who opens it in O_RDONLY mode is
considered a subscriber.

The advertiser writes to the node to publish data, while the subscriber reads
from the node to subscribe to data. Both parties can use poll to monitor the
node to receive events of interest. Control over device nodes by advertisers
and subscribers is achieved through ioctl operations.

**Code Location**
^^^^^^^^^^^^^^^^^

The directory apps/system/uorb contains the uORB wrapper, unit tests, physical
sensor topic definitions, and the uorb_listener tool.

::

  ├── Kconfig
  ├── listener.c   // Implementation of uorb_listener.c
  ├── Make.defs
  ├── Makefile
  ├── sensor       // Definitions for physical sensor topics
  │   ├── accel.c
  │   ├── accel.h
  │   ├── baro.c
  │   ├── baro.h
  │   ├── cap.c
  │   ├── cap.h
  │   ├── ...c
  │   ├── tvoc.c
  │   ├── tvoc.h
  │   ├── uv.c
  │   └── uv.h
  ├── test
  │   ├── unit_test.c   // Unit tests for uORB
  │   ├── utility.c
  │   └── utility.h
  └── uORB
      ├── uORB.c   // Implementation of uORB wrapper
      └── uORB.h

**Introduction to Key Data Structures**
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Each uORB topic corresponds to a ``struct orb_metadata`` structure, which is used to
describe the metadata of the topic, including the name o_name, the message size o_size,
and the debug format pointer o_format.

::

  struct orb_metadata
  {
    FAR const char   *o_name;       // Name of the topic
    uint16_t          o_size;       // Size of the message
    #ifdef CONFIG_DEBUG_UORB
    FAR const char   *o_format;     //  Format string used for structure input and output
    #endif
  };

::

  typedef FAR const struct orb_metadata *orb_id_t; // Pointer to orb_metadata as an identifier for uORB topics

After advertising or subscribing to each uORB topic, the caller can obtain the topic's
state by calling orb_get_state, which includes the current maximum publication frequency
max_frequency, the minimum batch interval min_batch_interval, the length of the internal
circular queue queue_size, the number of subscribers nsubscribers, and the generation index
for the main thread generation.

::

  struct orb_state              
  {                             
    uint32_t max_frequency;     // Maximum publication frequency
    uint32_t min_batch_interval;// Minimum batch interval
    uint32_t queue_size;        // Length of the internal circular queue
    uint32_t nsubscribers;      // Number of subscribers
    uint64_t generation;        // Generation index for the main thread
  };

uORB supports the instantiation of a topic into multiple entities, each with a corresponding
instance number starting from 0 and incrementing. struct orb_object contains information about
one such entity: its metadata meta and instance number instance.

::

  struct orb_object
  {
    orb_id_t meta;      // Pointer to the metadata of the topic
    int      instance;  // Instance number of the topic entity
  };

**Topic Definition**
^^^^^^^^^^^^^^^^^^^^

A large number of uORB topics are defined in PX4 at https://docs.px4.io/main/en/middleware/uorb_graph.html.

Defining a new topic involves creating the topic's data structure, declaring a global variable for
the topic's metadata, and optionally defining a debug output function for the topic's data.
Three macros are frequently used in the definition process:

``ORB_ID``: Used to obtain the global metadata handle for the topic.

``ORB_DECLARE``: Used to declare the global metadata for the topic.

``ORB_DEFINE``: Used to define the global metadata for the topic.

**API Description**
^^^^^^^^^^^^^^^^^^^

The uORB has a total of 30 APIs, which can be categorized into four groups: advertiser
class, subscriber class, general API class, and tool class. Almost all of these APIs
operate using file descriptors, so special attention should be paid to avoid using
them across processes (tasks).

**Advertise Class APIs**
------------------------

**Advertise Topic for Notification Types**
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This category includes a total of 5 APIs, among which orb_advertise,
orb_advertise_queue, and orb_advertise_multi are internally implemented based on
orb_advertise_multi_queue. They are permutations and combinations of the parameters
data, instance, and queue_size.

``orb_advertise_multi_queue`` requires specifying the topic metadata meta, the initial
data data, the instance instance, and the internal queue size when advertising a
topic. Upon success, it returns a file descriptor; upon failure, it returns -1 and
sets errno.

The instance is a pointer to an input parameter. If it is NULL, the instance will
increment from its existing value. Otherwise, the content pointed to by \*instance
will be used for advertising.

Multiple advertisements for the same topic instance are supported, allowing for
multiple publishers on a single device node.

::

  int orb_advertise_multi_queue(FAR const struct orb_metadata *meta,
                                FAR const void *data,               
                                FAR int *instance,                  
                                unsigned int queue_size);  
                                
  static inline int orb_advertise(FAR const struct orb_metadata *meta,
                                  FAR const void *data)
  {
    int instance = 0;
  
    return orb_advertise_multi_queue(meta, data, &instance, 1);
  }     
  
  static inline int orb_advertise_queue(FAR const struct orb_metadata *meta,
                                        FAR const void *data,
                                        unsigned int queue_size)
  {
    int instance = 0;
  
    return orb_advertise_multi_queue(meta, data, &instance, queue_size);
  }
  
  static inline int orb_advertise_multi(FAR const struct orb_metadata *meta,
                                        FAR const void *data,
                                        FAR int *instance)
  {
    return orb_advertise_multi_queue(meta, data, instance, 1);
  }

``orb_advertise_multi_queue_persist`` shares the same parameters with
``orb_advertise_multi_queue``, but with a different internal implementation.

::

  int orb_advertise_multi_queue_persist(FAR const struct orb_metadata *meta,
                                        FAR const void *data,
                                        FAR int *instance,
                                        unsigned int queue_size);

**Unadvertise a Topic**
~~~~~~~~~~~~~~~~~~~~~~~

The ``orb_unadvertise`` function takes the file descriptor fd returned by a topic
advertisement as its parameter and internally calls orb_close to achieve unadvertisement.

::

  static inline int orb_unadvertise(int fd)
  {
    return orb_close(fd);
  }

**Publishing Topic Data**
~~~~~~~~~~~~~~~~~~~~~~~~~

The ``orb_publish`` function takes the topic metadata meta, the advertisement handle fd,
and a pointer to the data data to be published as its parameters. It can only publish
one piece of data each time. In contrast, orb_publish_multi allows batch publishing.
The return values of the two functions differ: orb_publish returns 0 upon success and
-1 upon failure, setting errno in the latter case, while orb_publish_multi returns the
length of data published upon success.

::

  ssize_t orb_publish_multi(int fd, FAR const void *data, size_t len);
  
  static inline int orb_publish(FAR const struct orb_metadata *meta,
                                int fd, FARorb_close const void *data)
  {
    int ret;
  
    ret = orb_publish_multi(fd, data, meta->o_size);
    return ret == meta->o_size ? 0 : -1;
  }
  
  static inline int orb_publish_auto(FAR const struct orb_metadata *meta,
                                     FAR int *fd, FAR const void *data,
                                     FAR int *instance)；

**Subscribe Class APIs**
------------------------

**Subscribe to a Topic**
~~~~~~~~~~~~~~~~~~~~~~~~

The ``orb_subscribe`` function is internally implemented by orb_subscribe_multi,
with the main difference between them being whether an instance needs to be
specified for the subscription. Upon successful subscription, it returns an
fd (file descriptor); upon failure, it returns -1 and sets errno.

::

  int orb_subscribe_multi(FAR const struct orb_metadata *meta,
                          unsigned instance);
  
  static inline int orb_subscribe(FAR const struct orb_metadata *meta)
  {
    return orb_subscribe_multi(meta, 0);
  }
  
**Unsubscribe**
~~~~~~~~~~~~~~~

The ``orb_unsubscribe`` function takes the fd (file descriptor) returned by a
subscription as its parameter and internally calls orb_close to unsubscribe.

::

  static inline int orb_unsubscribe(int fd)
  {
    return orb_close(fd);
  }

**Retrieve Data**
~~~~~~~~~~~~~~~~~

The ``orb_copy`` function takes the topic metadata meta, the subscription handle
fd, and a pointer to the buffer buffer where the data will be stored as its
parameters. It can only read one piece of data each time. In contrast, 
orb_copy_multi allows batch reading. The return values of the two functions
differ: orb_copy returns 0 upon success and -1 upon failure, setting errno in
the latter case, while orb_copy_multi returns the length of data read upon success.

::

  ssize_t orb_copy_multi(int fd, FAR void *buffer, size_t len);
  
  static inline int orb_copy(FAR const struct orb_metadata *meta,
                             int fd, FAR void *buffer)
  {
    int ret;
  
    ret = orb_copy_multi(fd, buffer, meta->o_size);
    return ret == meta->o_size ? 0 : -1;
  }

**Normal Class APIs**
---------------------

**Open/Close Device Nodes**
~~~~~~~~~~~~~~~~~~~~~~~~~~~

``orb_open`` is used to open a character device node. The parameters include name
(the topic name), instance (the topic instance index), and flags (the open mode).
Subscribers typically open with O_RDONLY, publishers with O_WRONLY, and a third
party can open the node with "0" to retrieve device node information.
It corresponds to ``orb_close`` for closing the node.

::

  int orb_open(FAR const char *name, int instance, int flags);
  int orb_close(int fd);

**Retrieve Topic Status**
~~~~~~~~~~~~~~~~~~~~~~~~~

``orb_get_state`` this function or method is not explicitly named in your list but
is implied by the context.)

::

  int orb_get_state(int fd, FAR struct orb_state *state);

**Check for Updates**
~~~~~~~~~~~~~~~~~~~~~

``orb_check`` is used to check if there is new data for the current topic.
It takes a pointer to an updated variable as an input parameter.

::

  int orb_check(int fd, FAR bool *updated);

**Control the Topic**
~~~~~~~~~~~~~~~~~~~~~

Applications can use ``orb_ioctl`` to control physical sensor topics,
such as adjusting the range, resolution, etc., of accelerometers,
gyroscopes, magnetometers, and PPG sensors.

::

  int orb_ioctl(int fd, int cmd, unsigned long arg);

**Set/Get Topic Batch Parameters**
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``orb_set_batch_interval``/``orb_get_batch_interval`` are used to set/get
the maximum delay reporting time for a topic, in microseconds (μs).
This API is only applicable to physical sensors with hardware FIFO support.

::

  int orb_set_batch_interval(int fd, unsigned batch_interval);
  int orb_get_batch_interval(int fd, FAR unsigned *batch_interval);

**Set/Get Topic Interval Parameters**
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Frequency and sampling interval are reciprocals of each other.
``orb_set_interval``/``orb_get_interval`` are used to set/get the sampling rate,
in microseconds (μs), while ``orb_set_frequency``/``orb_get_frequency`` are used to
set/get the sampling frequency, in Hertz (Hz).

::

  int orb_set_interval(int fd, unsigned interval);
  int orb_get_interval(int fd, FAR unsigned *interval);
  static inline int orb_set_frequency(int fd, unsigned frequency)
  static inline int orb_get_frequency(int fd, FAR unsigned *frequency)

**orb_flush**
~~~~~~~~~~~~~

``orb_flush`` supports the flush operation for topics with hardware FIFO. After the flush
operation is completed, a POLLPRI event will be generated for the fd, and
then orb_get_events can be called to retrieve the corresponding event.

::

  int orb_flush(int fd);

**orb_get_events**
~~~~~~~~~~~~~~~~~~

``orb_get_events`` retrieves events. Currently, the supported events include ORB_EVENT_FLUSH_COMPLETE.

::

  int orb_get_events(int fd, FAR unsigned int *events);

**Tool Class APIs**
-------------------

**Check if a Publisher Exists for a Topic**
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``orb_exists`` is used to check if there is a publisher for a topic. It takes the topic
metadata meta and the topic instance index instance as parameters. It returns 0 if the
check is successful and an ERROR code if it fails.

::
  
  int orb_exists(FAR const struct orb_metadata *meta, int instance);


**Timestamp Calculation**
~~~~~~~~~~~~~~~~~~~~~~~~~

``orb_absolute_time`` returns the current timestamp.
``orb_elapsed_time`` returns the difference between two timestamps.

::

  orb_abstime orb_absolute_time(void);
  static inline orb_abstime orb_elapsed_time(FAR const orb_abstime *then)

**Get the Number of Topic Instances**
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``orb_group_count`` returns the number of instances for a specific topic.

::

  int orb_group_count(FAR const struct orb_metadata *meta);

**Get Topic Metadata**
~~~~~~~~~~~~~~~~~~~~~~

``orb_get_meta`` is used to retrieve a pointer to the topic metadata by using a string.
Currently, this function has significant limitations: for non-physical sensors,
you must explicitly subscribe to or advertise the topic to successfully obtain
the metadata pointer.

::

  FAR const struct orb_metadata *orb_get_meta(FAR const char *name);

These tool class APIs provide additional utilities for working with ORB
(Object Request Broker) topics, allowing for tasks such as checking the existence
of publishers, calculating timestamps, retrieving the number of topic instances, and
accessing topic metadata. These functions are crucial for managing and interacting with
topics in an ORB-based system.

**Usage in NuttX**
^^^^^^^^^^^^^^^^^^

In NuttX, all physical sensor drivers automatically advertise their topics upon
system startup. The sensors are then controlled to turn on or off based on whether
any applications have subscribed to the related topics, enabling intelligent
low-power consumption management.

All virtual topics (algorithms, states, cross-core topics) automatically register
character device nodes upon their first publication or subscription. Once registered,
these nodes remain active even if the publication or subscription is later canceled.

When subscribers and publishers in an application need to monitor each other's status,
the poll function is used for status notifications. Subscribers and publishers synchronize
their status via the POLLPRI signal. When a publisher publishes data, a POLLIN event is
generated to notify all subscribers. POLLPRI events are triggered in the following scenarios:

When a new subscriber or publisher joins the topic.

When a subscriber sets the sampling rate or batch parameters for the topic.

When a subscriber or publisher leaves the topic.

Whenever a POLLPRI event occurs, you can call orb_get_state to retrieve the current status
of the topic, including the maximum publishing frequency (max_frequency), the minimum batch
interval (min_batch_interval), the internal ring buffer size (queue_size), the number of
subscribers (nsubscribers), and the data generation count.

In summary, if subscribers and publishers are interdependent, it is recommended to use a
polling or libuv-based programming structure. If they are independent, a notification-based
topic approach is preferred.

**Fusion Algorithm Models**
---------------------------

If multiple applications are interconnected, with one serving as
the input and another as the output, shared topics can be established to facilitate decoupling
between applications. For example, a calibration algorithm module can subscribe to an uncalibrated
sensor data topic and publish calibrated sensor data. A motion algorithm module can subscribe to
both calibrated and uncalibrated data to generate algorithmic topics, with multiple applications
ignoring each other's existence.

**Subscriber and Publisher Monitoring**
---------------------------------------

Subscribers can check for update events via POLLIN events,
while publishers can monitor changes in subscriber count, sampling rates, and other statuses via
POLLPRI events. The optimal status can be obtained using orb_get_state.
By leveraging these mechanisms, NuttX provides a robust framework for efficient data communication
and management in an embedded system environment.

**Tools**
^^^^^^^^^

**uorb_listener**
-----------------

``uorb_listener`` is a testing tool located above the uORB layer. It calls the uORB API to subscribe
to and obtain topic information, further verifying whether the underlying system is functioning
correctly. uorb_listener only monitors topics that have been advertised. The entire listening process
can be paused using Ctrl+C. Usage instructions can be viewed by running uorb_listener -h.
Below are some commonly used cases:

::

  listener <command> [arguments...]
   Commands:
          <topics_name> Topic name. Multi name are separated by ','
          [-h       ]  Listener commands help
          [-f       ]  Record uorb data to file
          [-n <val> ]  Number of messages, default: 0
          [-r <val> ]  Subscription rate (unlimited if 0), default: 0
          [-b <val> ]  Subscription maximum report latency in us(unlimited if 0), default: 0
          [-t <val> ]  Time of listener, in seconds, default: 5
          [-T       ]  Top, continuously print updating objects
          [-l       ]  Top only execute once.
        

``uorb_listener`` continuously prints information for all topics at the frequency of topic publications.

``uorb_listener -f`` continuously saves information for all topics to /data/uorb/***/***.csv at the frequency of
topic publications. If the -f flag is used and the file cannot be created, the data will be output to the terminal instead.

``uorb_listener -f sensor_accel0`` continuously saves information for the specified topic to a file at the frequency of topic publications.

``uorb_listener n 1`` prints a snapshot of the current information for all topics.

``uorb_listener n`` num prints information for all topics until num messages have been received, at the frequency of topic publications.

``uorb_listener r 1`` prints information for all topics at a frequency of 1Hz.

``uorb_listener r x n`` num prints information for all topics at a frequency of xHz until num messages have been received.

``uorb_listener [specified_topic_list] r 1`` continuously prints information for the specified topics at a
frequency of 1Hz. In the specified topic list, topics are separated by commas (,). Each entry can be a topic name,
such as sensor_accel, which will print information for all instances of that topic. It can also be a topic instance name,
such as sensor_mag0, which will only print information for that specific topic instance.

This tool provides a flexible way to monitor and log uORB topic data, aiding in the debugging and verification of the system's behavior.


**Generator Debugging Tool Instructions**
-----------------------------------------

``uorb_generator`` this tool can be used in conjunction with ``uorb_listener``.

Before using the tool, it is necessary to set the CONFIG_LINE_MAX parameter to a sufficiently long length to ensure that the terminal
can accept complete input data. A recommendation is to set it to 256 or 512.

Incoming data can be printed via uorb_listener or concatenated manually using format information, but it must be
ensured that the string and struct information are consistent. Topics saved using uorb_listener -f can be pulled and
imported into the simulator for debugging (mount -t hostfs -o fs=/home/xxx/ /data).

**Parameter Description:**

``-f`` specifies the path to the input playback file.
``-n`` specifies the number of times to playback the data. This option is only effective when -s is enabled.
``-r`` specifies the playback frequency (in HZ, e.g., 5hz, 20hz). This option is only effective when -s is enabled.
``-t`` specifies the topic for playback, with the option to specify a specific instance value afterward.
``-s`` enables playback of simulated (fake) data, generating struct data from input entered via the terminal. It will modify the timestamp of the current data to real-time. Simulated data should be placed at the end of the entire command.

By following these instructions, users can effectively utilize the Generator debugging tool in conjunction with uorb_listener for system debugging and verification.

::

  The tool publishes topic data via uorb.
  Notice:LINE_MAX must be set to 128 or more.
  
  generator <command> [arguments...]
    Commands:
      <topics_name> The playback topic name.
      [-h       ]  Listener commands help.
      [-f <val> ]  File path to be played back(absolute path).
      [-n <val> ]  Number of playbacks(fake model), default: 1
      [-r <val> ]  The rate for playing fake data is only valid when parameter 's' is used. 
                   default:10hz.
      [-s <val> ]  Playback fake data.
      [-t <val> ]  Playback topic.
       e.g.:
          sim - sensor_accel0:
            uorb_generator -n 100 -r 5 -s -t sensor_accel0 timestamp:23191100,x:0.1,y:9.7,z:0.81,temperature:22.15
  
          sim - sensor_baro0:
            uorb_generator -n 100 -r 5 -s -t sensor_baro0 timestamp:23191100,pressure:999.12,temperature:26.34
  
          fies - sensor_accel1
          uorb_generator -f /data/uorb/20240823061723/sensor_accel0.csv -t sensor_accel1

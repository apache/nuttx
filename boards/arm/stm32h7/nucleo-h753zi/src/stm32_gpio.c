/****************************************************************************
 * boards/arm/stm32h7/nucleo-h753zi/src/stm32_gpio.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/clock.h>
#include <nuttx/wdog.h>
#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "chip.h"
#include "stm32_gpio.h"
#include "nucleo-h753zi.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO Driver Validation and Debug Macros */

#define STM32_GPIO_VALIDATE_DEVICE(dev) \
  do { \
    DEBUGASSERT((dev) != NULL); \
  } while (0)

#define STM32_GPIO_VALIDATE_VALUE_PTR(value) \
  do { \
    DEBUGASSERT((value) != NULL); \
  } while (0)

#define STM32_GPIO_VALIDATE_INDEX(id, max) \
  do { \
    DEBUGASSERT((id) < (max)); \
  } while (0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Standard GPIO device structure
 *
 * This structure represents a basic GPIO pin device that can be used
 * for either input or output operations. It contains the standard NuttX
 * GPIO device structure plus an ID field to identify which specific
 * GPIO pin this device represents in the board's GPIO arrays.
 */

struct stm32gpio_dev_s
{
  struct gpio_dev_s gpio;    /* Standard NuttX GPIO device structure */
  uint8_t id;                /* Index into the GPIO configuration arrays */
};

/* GPIO interrupt device structure
 *
 * This structure extends the basic GPIO device to add interrupt capability.
 * It includes a callback function pointer that will be invoked when the
 * GPIO interrupt is triggered. The structure embeds the basic GPIO device
 * rather than inheriting from it to maintain compatibility with NuttX
 * device management.
 */

struct stm32gpint_dev_s
{
  struct stm32gpio_dev_s stm32gpio;  /* Base GPIO device structure */
  pin_interrupt_t callback;          /* User-provided interrupt callback */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* GPIO Input Operations */

static int stm32_gpin_read(struct gpio_dev_s *dev, bool *value);

/* GPIO Output Operations */

static int stm32_gpout_read(struct gpio_dev_s *dev, bool *value);
static int stm32_gpout_write(struct gpio_dev_s *dev, bool value);

/* GPIO Interrupt Operations */

static int stm32_gpint_read(struct gpio_dev_s *dev, bool *value);
static int stm32_gpint_attach(struct gpio_dev_s *dev,
                              pin_interrupt_t callback);
static int stm32_gpint_enable(struct gpio_dev_s *dev, bool enable);

/* Interrupt Service Routine */

static int stm32_gpio_interrupt_handler(int irq, void *context, void *arg);

/* Modular Initialization Functions */

static int stm32_gpio_init_input_pins(int *pincount);
static int stm32_gpio_init_output_pins(int *pincount);
static int stm32_gpio_init_interrupt_pins(int *pincount);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* GPIO Operations Tables
 *
 * These structures define the operations available for each type of GPIO.
 * Each GPIO type (input, output, interrupt) has different capabilities:
 * - Input: Only read operations
 * - Output: Read and write operations
 * - Interrupt: Read, attach callback, and enable/disable interrupt
 */

static const struct gpio_operations_s g_gpin_ops =
{
  .go_read   = stm32_gpin_read,
  .go_write  = NULL,            /* Input pins cannot be written */
  .go_attach = NULL,            /* Regular inputs don't support interrupts */
  .go_enable = NULL,
};

static const struct gpio_operations_s g_gpout_ops =
{
  .go_read   = stm32_gpout_read,
  .go_write  = stm32_gpout_write,
  .go_attach = NULL,              /* Output pins don't support interrupts */
  .go_enable = NULL,
};

static const struct gpio_operations_s g_gpint_ops =
{
  .go_read   = stm32_gpint_read,
  .go_write  = NULL,                 /* Interrupt pins are input-only */
  .go_attach = stm32_gpint_attach,
  .go_enable = stm32_gpint_enable,
};

/* GPIO Pin Configuration Arrays and Device Instances
 *
 * These arrays map the logical GPIO numbers used by applications to the
 * actual STM32 GPIO pin configurations. The configurations are defined
 * in the board header file (nucleo-h753zi.h).
 */

#if BOARD_NGPIOIN > 0
static const uint32_t g_gpioinputs[BOARD_NGPIOIN] =
{
  GPIO_BTN_BUILT_IN,  /* User button on PC13 - configured as input */
};

static struct stm32gpio_dev_s g_gpin_devices[BOARD_NGPIOIN];
#endif

#if BOARD_NGPIOOUT > 0
static const uint32_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
  GPIO_LD1,  /* Green LED on PB0 - configured as output */
  GPIO_LD2,  /* Orange LED on PE1 - configured as output */
  GPIO_LD3,  /* Red LED on PB14 - configured as output */
};

static struct stm32gpio_dev_s g_gpout_devices[BOARD_NGPIOOUT];
#endif

#if BOARD_NGPIOINT > 0
static const uint32_t g_gpiointinputs[BOARD_NGPIOINT] =
{
  GPIO_INT1,  /* Custom interrupt pin - define in board header */
};

static struct stm32gpint_dev_s g_gpint_devices[BOARD_NGPIOINT];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_gpio_interrupt_handler
 *
 * Description:
 *   This is the common interrupt service routine for all GPIO interrupt
 *   pins.
 *
 *   When a GPIO interrupt occurs, the STM32 interrupt controller will call
 *   this function with the appropriate arguments. This function then calls
 *   the user-provided callback function that was registered via the attach
 *   operation.
 *
 *   The function performs minimal processing in interrupt context - it just
 *   validates the parameters and calls the user callback. All complex
 *   processing should be done in the callback function, preferably by
 *   posting to a work queue or semaphore.
 *
 * Input Parameters:
 *  irq     -The interrupt request number (not used in this implementation)
 *  context -Saved processor context (not used in this implementation)
 *  arg     -Pointer to the stm32gpint_dev_s structure for the triggering pin
 *
 * Returned Value:
 *   Always returns OK to indicate the interrupt was handled
 *
 * Assumptions:
 *   - Called in interrupt context, so keep processing minimal
 *   - The arg parameter points to a valid stm32gpint_dev_s structure
 *   - The callback function pointer is valid (checked by caller)
 *
 ****************************************************************************/

static int stm32_gpio_interrupt_handler(int irq, void *context, void *arg)
{
  struct stm32gpint_dev_s *stm32gpint = (struct stm32gpint_dev_s *)arg;

  DEBUGASSERT(stm32gpint != NULL && stm32gpint->callback != NULL);

  gpioinfo("GPIO interrupt %d triggered, calling callback %p\n",
           stm32gpint->stm32gpio.id, stm32gpint->callback);

  /* Execute the user-registered callback function.
   * The callback receives the GPIO device pointer and the pin ID.
   */

  stm32gpint->callback(&stm32gpint->stm32gpio.gpio,
                       stm32gpint->stm32gpio.id);
  return OK;
}

/****************************************************************************
 * Name: stm32_gpin_read
 *
 * Description:
 *   Read the current logical state of a GPIO input pin.
 *
 *   This function reads the physical state of a GPIO pin configured as an
 *   input and returns the logical value (true for HIGH, false for LOW).
 *   The pin configuration (pull-up, pull-down, etc.) affects what voltage
 *   levels are interpreted as HIGH or LOW.
 *
 *   For the Nucleo-H753ZI, this is typically used to read the user button
 *   state. The button is active-low (pressed = LOW voltage), but the
 *   logical interpretation depends on how the pin is configured in the
 *   board definitions.
 *
 * Input Parameters:
 *   dev   - Pointer to the GPIO device structure
 *   value - Pointer to store the read value (true = HIGH, false = LOW)
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   - The GPIO pin was previously configured as an input
 *   - The device structure is valid and properly initialized
 *   - The pin ID is within the valid range for input pins
 *
 ****************************************************************************/

static int stm32_gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct stm32gpio_dev_s *stm32gpio = (struct stm32gpio_dev_s *)dev;

  /* Validate all input parameters */

  STM32_GPIO_VALIDATE_DEVICE(stm32gpio);
  STM32_GPIO_VALIDATE_VALUE_PTR(value);
  STM32_GPIO_VALIDATE_INDEX(stm32gpio->id, BOARD_NGPIOIN);

  gpioinfo("Reading GPIO input pin %d\n", stm32gpio->id);

  /* Read the current state of the input pin */

  *value = stm32_gpioread(g_gpioinputs[stm32gpio->id]);

  gpioinfo("GPIO input %d state: %s\n",
           stm32gpio->id, *value ? "HIGH" : "LOW");

  return OK;
}

/****************************************************************************
 * Name: stm32_gpout_read
 *
 * Description:
 *   Read the current logical state of a GPIO output pin.
 *
 *   This function reads back the current state that was written to an output
 *   pin. This is useful for applications that need to know the current state
 *   of an output (for example, to toggle a LED state). The function reads
 *   the actual pin state, not just the last written value, so it reflects
 *   the real electrical state of the pin.
 *
 *   For the Nucleo-H753ZI LEDs, this allows applications to check if a LED
 *   is currently on or off before deciding whether to toggle it.
 *
 * Input Parameters:
 *   dev   - Pointer to the GPIO device structure
 *   value - Pointer to store the read value (true = HIGH, false = LOW)
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   - The GPIO pin was previously configured as an output
 *   - The device structure is valid and properly initialized
 *   - The pin ID is within the valid range for output pins
 *
 ****************************************************************************/

static int stm32_gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct stm32gpio_dev_s *stm32gpio = (struct stm32gpio_dev_s *)dev;

  /* Validate all input parameters */

  STM32_GPIO_VALIDATE_DEVICE(stm32gpio);
  STM32_GPIO_VALIDATE_VALUE_PTR(value);
  STM32_GPIO_VALIDATE_INDEX(stm32gpio->id, BOARD_NGPIOOUT);

  gpioinfo("Reading GPIO output pin %d current state\n", stm32gpio->id);

  /* Read the current output state of the pin */

  *value = stm32_gpioread(g_gpiooutputs[stm32gpio->id]);

  gpioinfo("GPIO output %d current state: %s\n",
           stm32gpio->id, *value ? "HIGH" : "LOW");

  return OK;
}

/****************************************************************************
 * Name: stm32_gpout_write
 *
 * Description:
 *   Write a logical value to a GPIO output pin.
 *
 *   This is the core function that allows applications to control GPIO
 *   output pins. It sets the electrical state of the pin to either HIGH
 *   (3.3V) or LOW (0V) based on the boolean value provided.
 *
 *   For the Nucleo-H753ZI LEDs:
 *   - Writing true (HIGH) will turn ON the LED (LED is active-high)
 *   - Writing false (LOW) will turn OFF the LED
 *
 *   The function provides debug output to help with troubleshooting GPIO
 *   operations during development. In production builds with debug disabled,
 *   this overhead is eliminated by the compiler.
 *
 *   This function is called by the NuttX GPIO framework when user
 *   applications perform write operations via ioctl(GPIOC_WRITE)
 *   or gpio_write() calls.
 *
 * Input Parameters:
 *   dev   - Pointer to the GPIO device structure
 *   value - Value to write (true = HIGH/3.3V, false = LOW/0V)
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   - The GPIO pin was previously configured as an output
 *   - The device structure is valid and properly initialized
 *   - The pin ID is within the valid range for output pins
 *   - The pin is not being used by another driver or function
 *
 ****************************************************************************/

static int stm32_gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct stm32gpio_dev_s *stm32gpio = (struct stm32gpio_dev_s *)dev;

  /* Validate input parameters */

  STM32_GPIO_VALIDATE_DEVICE(stm32gpio);
  STM32_GPIO_VALIDATE_INDEX(stm32gpio->id, BOARD_NGPIOOUT);

  gpioinfo("Writing GPIO output pin %d: %s\n",
           stm32gpio->id, value ? "HIGH" : "LOW");

  /* Write the new state to the GPIO pin.
   * This calls the STM32-specific GPIO write function which handles
   * the low-level register manipulation to set the pin state.
   */

  stm32_gpiowrite(g_gpiooutputs[stm32gpio->id], value);

  gpioinfo("GPIO output %d write completed successfully\n", stm32gpio->id);

  return OK;
}

/****************************************************************************
 * Name: stm32_gpint_read
 *
 * Description:
 *   Read the current logical state of a GPIO interrupt pin.
 *
 *   This function allows reading the current state of a pin that is
 *   configured for interrupt generation. Even though the pin is primarily
 *   used for interrupts, it can still be read synchronously to check its
 *   current state. This is useful for:
 *
 *   - Polling the pin state when interrupts are disabled
 *   - Checking initial pin state during initialization
 *   - Debouncing in software by reading multiple times
 *   - Diagnostics and debugging
 *
 *   Note that this reads the actual pin electrical state, which may be
 *   different from the last interrupt trigger if the pin has changed
 *   since the interrupt occurred.
 *
 * Input Parameters:
 *   dev   - Pointer to the GPIO device structure
 *   value - Pointer to store the read value (true = HIGH, false = LOW)
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   - The GPIO pin was previously configured for interrupt input
 *   - The device structure is valid and properly initialized
 *   - The pin ID is within the valid range for interrupt pins
 *
 ****************************************************************************/

static int stm32_gpint_read(struct gpio_dev_s *dev, bool *value)
{
  struct stm32gpint_dev_s *stm32gpint = (struct stm32gpint_dev_s *)dev;

  /* Validate all input parameters */

  STM32_GPIO_VALIDATE_DEVICE(stm32gpint);
  STM32_GPIO_VALIDATE_VALUE_PTR(value);
  STM32_GPIO_VALIDATE_INDEX(stm32gpint->stm32gpio.id, BOARD_NGPIOINT);

  gpioinfo("Reading GPIO interrupt pin %d current state\n",
           stm32gpint->stm32gpio.id);

  /* Read the current state of the interrupt pin */

  *value = stm32_gpioread(g_gpiointinputs[stm32gpint->stm32gpio.id]);

  gpioinfo("GPIO interrupt pin %d current state: %s\n",
           stm32gpint->stm32gpio.id, *value ? "HIGH" : "LOW");

  return OK;
}

/****************************************************************************
 * Name: stm32_gpint_attach
 *
 * Description:
 *   Attach a callback function to a GPIO interrupt pin.
 *
 *   This function registers a user-provided callback function that will be
 *   executed when the GPIO interrupt is triggered. The callback is stored
 *   in the device structure and will be called by the interrupt handler.
 *
 *   Key behaviors:
 *   - Any existing interrupt is automatically disabled during attachment
 *   - The callback can be changed by calling attach again with a new
 *     function.
 *
 *   - Setting callback to NULL effectively detaches the interrupt
 *   - The interrupt must be explicitly enabled after attachment using
 *     the enable operation
 *
 *   The callback function will be called in interrupt context, so it should:
 *   - Be as fast as possible
 *   - Not call blocking functions
 *   - Consider posting to work queues for complex processing
 *   - Be reentrant if the same callback is used for multiple pins
 *
 * Input Parameters:
 *   dev      - Pointer to the GPIO device structure
 *   callback - Function to call when interrupt occurs
 *              (can be NULL to detach)
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   - The GPIO pin was previously configured for interrupt input
 *   - The device structure is valid and properly initialized
 *   - The callback function (if not NULL) is valid and reentrant
 *
 ****************************************************************************/

static int stm32_gpint_attach(struct gpio_dev_s *dev,
                              pin_interrupt_t callback)
{
  struct stm32gpint_dev_s *stm32gpint = (struct stm32gpint_dev_s *)dev;

  STM32_GPIO_VALIDATE_DEVICE(stm32gpint);

  gpioinfo("Attaching callback %p to GPIO interrupt pin %d\n",
           callback, stm32gpint->stm32gpio.id);

  /* Disable any existing interrupt configuration before changing callback.
   * This ensures we don't get spurious interrupts during the transition.
   */

  stm32_gpiosetevent(g_gpiointinputs[stm32gpint->stm32gpio.id],
                     false, false, false, NULL, NULL);

  /* Store the new callback function */

  stm32gpint->callback = callback;

  if (callback != NULL)
    {
      gpioinfo("Callback attached successfully to GPIO interrupt %d\n",
               stm32gpint->stm32gpio.id);
    }
  else
    {
      gpioinfo("Callback detached from GPIO interrupt %d\n",
               stm32gpint->stm32gpio.id);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_gpint_enable
 *
 * Description:
 *   Enable or disable GPIO interrupt generation.
 *
 *   This function controls whether the GPIO pin will actually generate
 *   interrupts when the configured event occurs. The pin must have a
 *   callback attached before enabling interrupts.
 *
 *   When enabling:
 *   - Configures the pin for rising edge detection (button release)
 *   - Registers the interrupt handler with the STM32 interrupt controller
 *   - The interrupt will fire when the pin transitions from LOW to HIGH
 *
 *   When disabling:
 *   - Removes interrupt configuration from the pin
 *   - No interrupts will be generated until re-enabled
 *   - The callback remains attached and can be re-enabled later
 *
 *   For the Nucleo board button (if used as interrupt):
 *   - Button is typically active-low (pressed = LOW)
 *   - Rising edge = button release
 *   - Falling edge = button press
 *   - Configure based on desired behavior
 *
 * Input Parameters:
 *   dev    - Pointer to the GPIO device structure
 *   enable - true to enable interrupts, false to disable
 *
 * Returned Value:
 *   OK on success; -EINVAL if no callback is attached when enabling
 *
 * Assumptions:
 *   - The GPIO pin was previously configured for interrupt input
 *   - For enable=true: a valid callback must be attached first
 *   - The device structure is valid and properly initialized
 *
 ****************************************************************************/

static int stm32_gpint_enable(struct gpio_dev_s *dev, bool enable)
{
  struct stm32gpint_dev_s *stm32gpint = (struct stm32gpint_dev_s *)dev;

  STM32_GPIO_VALIDATE_DEVICE(stm32gpint);

  if (enable)
    {
      /* Cannot enable interrupt without a callback function */

      if (stm32gpint->callback == NULL)
        {
          gpioerr("ERROR: Cannot enable GPIO interrupt %d \n",
                  stm32gpint->stm32gpio.id);
          return -EINVAL;
        }

      gpioinfo("Enabling GPIO interrupt %d with rising edge detection\n",
               stm32gpint->stm32gpio.id);

      /* Configure the interrupt for rising edge detection.
       * Parameters: pin, rising, falling, filter, handler, arg
       * - rising=true: interrupt on LOW->HIGH transition
       * - falling=false: no interrupt on HIGH->LOW transition
       * - filter=false: no input filtering
       */

      stm32_gpiosetevent(g_gpiointinputs[stm32gpint->stm32gpio.id],
                         true, false, false,
                         stm32_gpio_interrupt_handler,
                         &g_gpint_devices[stm32gpint->stm32gpio.id]);

      gpioinfo("GPIO interrupt %d enabled successfully\n",
               stm32gpint->stm32gpio.id);
    }
  else
    {
      gpioinfo("Disabling GPIO interrupt %d\n", stm32gpint->stm32gpio.id);

      /* Disable the interrupt by clearing all event configuration */

      stm32_gpiosetevent(g_gpiointinputs[stm32gpint->stm32gpio.id],
                         false, false, false, NULL, NULL);

      gpioinfo("GPIO interrupt %d disabled successfully\n",
               stm32gpint->stm32gpio.id);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_gpio_init_input_pins
 *
 * Description:
 *   Initialize all GPIO input pins defined for this board.
 *
 *   This function sets up each input pin by:
 *   1. Initializing the device structure with proper type and operations
 *   2. Registering the pin with the NuttX GPIO framework
 *   3. Configuring the physical STM32 GPIO hardware
 *
 *   After this initialization, applications can access the input pins
 *   via the /dev/gpio device interface using standard read operations.
 *
 *   For Nucleo-H753ZI, this typically configures the user button (PC13)
 *   as a readable input that applications can poll.
 *
 * Input Parameters:
 *   pincount - Pointer to current pin count (updated by this function)
 *
 * Returned Value:
 *   Number of input pins initialized
 *
 * Side Effects:
 *   - Updates the pincount parameter with number of pins registered
 *   - Configures STM32 GPIO hardware registers
 *   - Registers devices with NuttX GPIO framework
 *
 ****************************************************************************/

static int stm32_gpio_init_input_pins(int *pincount)
{
#if BOARD_NGPIOIN > 0
  int i;
  int pins_initialized = 0;

  gpioinfo("Initializing %d GPIO input pins\n", BOARD_NGPIOIN);

  for (i = 0; i < BOARD_NGPIOIN; i++)
    {
      /* Initialize the GPIO device structure */

      g_gpin_devices[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin_devices[i].gpio.gp_ops     = &g_gpin_ops;
      g_gpin_devices[i].id              = i;

      /* Register the GPIO pin with the NuttX GPIO framework */

      gpio_pin_register(&g_gpin_devices[i].gpio, (*pincount)++);

      /* Configure the physical STM32 GPIO pin hardware */

      stm32_configgpio(g_gpioinputs[i]);

      pins_initialized++;

      gpioinfo("GPIO input %d: configured and registered as /dev/gpio%d\n",
               i, (*pincount) - 1);
    }

  gpioinfo("GPIO input initialization completed: %d pins\n",
           pins_initialized);
  return pins_initialized;
#else
  gpioinfo("No GPIO input pins configured for this board\n");
  return 0;
#endif
}

/****************************************************************************
 * Name: stm32_gpio_init_output_pins
 *
 * Description:
 *   Initialize all GPIO output pins defined for this board.
 *
 *   This function sets up each output pin by:
 *   1. Initializing the device structure with output type and operations
 *   2. Registering the pin with the NuttX GPIO framework
 *   3. Setting initial state to LOW (off) for safety
 *   4. Configuring the physical STM32 GPIO hardware
 *
 *   The initial LOW state ensures that LEDs start in the OFF state and
 *   other outputs start in a safe condition. Applications can then control
 *   the outputs via write operations.
 *
 *   For Nucleo-H753ZI, this configures the three onboard LEDs:
 *   - LD1 (Green LED on PB0)
 *   - LD2 (Orange LED on PE1)
 *   - LD3 (Red LED on PB14)
 *
 *   After initialization, applications can control these LEDs via
 *   /dev/gpio device interface using ioctl(GPIOC_WRITE) calls.
 *
 * Input Parameters:
 *   pincount - Pointer to current pin count (updated by this function)
 *
 * Returned Value:
 *   Number of output pins initialized
 *
 * Side Effects:
 *   - Updates the pincount parameter with number of pins registered
 *   - Sets all output pins to LOW state initially
 *   - Configures STM32 GPIO hardware registers
 *   - Registers devices with NuttX GPIO framework
 *
 ****************************************************************************/

static int stm32_gpio_init_output_pins(int *pincount)
{
#if BOARD_NGPIOOUT > 0
  int i;
  int pins_initialized = 0;

  gpioinfo("Initializing %d GPIO output pins\n", BOARD_NGPIOOUT);

  for (i = 0; i < BOARD_NGPIOOUT; i++)
    {
      /* Initialize the GPIO device structure */

      g_gpout_devices[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout_devices[i].gpio.gp_ops     = &g_gpout_ops;
      g_gpout_devices[i].id              = i;

      /* Register the GPIO pin with the NuttX GPIO framework */

      gpio_pin_register(&g_gpout_devices[i].gpio, (*pincount)++);

      /* Initialize output to LOW for safety, then configure hardware.
       * This ensures LEDs start OFF and other outputs start in safe state.
       */

      stm32_gpiowrite(g_gpiooutputs[i], false);
      stm32_configgpio(g_gpiooutputs[i]);

      pins_initialized++;

      gpioinfo("GPIO output %d: configured as LOW at /dev/gpio%d\n",
               i, (*pincount) - 1);
    }

  gpioinfo("GPIO output initialization completed: %d pins\n",
            pins_initialized);

  return pins_initialized;
#else
  gpioinfo("No GPIO output pins configured for this board\n");
  return 0;
#endif
}

/****************************************************************************
 * Name: stm32_gpio_init_interrupt_pins
 *
 * Description:
 *   Initialize all GPIO interrupt pins defined for this board.
 *
 *   This function sets up each interrupt-capable pin by:
 *   1. Initializing the extended device structure with interrupt capability
 *   2. Registering the pin with the NuttX GPIO framework
 *   3. Configuring the physical STM32 GPIO hardware for input
 *   4. Setting up initial state (callback = NULL, interrupt disabled)
 *
 *   Unlike simple input pins, interrupt pins require additional setup
 *   to support asynchronous event notification. The pin starts in a safe
 *   state with no callback attached and interrupts disabled. Applications
 *   must explicitly attach a callback and enable interrupts to receive
 *   asynchronous notifications.
 *
 *   The interrupt pins can also be read synchronously like regular inputs,
 *   providing flexibility in how applications interact with them.
 *
 *   Important: The actual interrupt configuration (edge detection, etc.)
 *   is done later when the interrupt is enabled, not during initialization.
 *
 * Input Parameters:
 *   pincount - Pointer to current pin count (updated by this function)
 *
 * Returned Value:
 *   Number of interrupt pins initialized
 *
 * Side Effects:
 *   - Updates the pincount parameter with number of pins registered
 *   - Configures STM32 GPIO hardware registers for input mode
 *   - Registers devices with NuttX GPIO framework
 *   - Initializes callback pointers to NULL for safety
 *
 ****************************************************************************/

static int stm32_gpio_init_interrupt_pins(int *pincount)
{
#if BOARD_NGPIOINT > 0
  int i;
  int pins_initialized = 0;

  gpioinfo("Initializing %d GPIO interrupt pins\n", BOARD_NGPIOINT);

  for (i = 0; i < BOARD_NGPIOINT; i++)
    {
      /* Initialize the GPIO interrupt device structure */

      g_gpint_devices[i].stm32gpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint_devices[i].stm32gpio.gpio.gp_ops     = &g_gpint_ops;
      g_gpint_devices[i].stm32gpio.id              = i;

      /* Safe initial state */

      g_gpint_devices[i].callback = NULL;

      /* Register the GPIO pin with the NuttX GPIO framework */

      gpio_pin_register(&g_gpint_devices[i].stm32gpio.gpio, (*pincount)++);

      /* Configure the physical STM32 GPIO pin for input.
       * Interrupt configuration will be done later when enabled.
       */

      stm32_configgpio(g_gpiointinputs[i]);

      pins_initialized++;

      gpioinfo("GPIO interrupt %d: registered as /dev/gpio%d\n",
               i, (*pincount) - 1);
    }

  gpioinfo("GPIO interrupt initialization completed: %d pins\n",
            pins_initialized);

  return pins_initialized;
#else
  gpioinfo("No GPIO interrupt pins configured for this board\n");
  return 0;
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_gpio_initialize
 *
 * Description:
 *   Initialize the GPIO driver for the STM32H7 Nucleo board.
 *
 *   This is the main initialization function that sets up all GPIO pins
 *   defined for the board. It coordinates the initialization of different
 *   types of GPIO pins (inputs, outputs, interrupts) and registers them
 *   with the NuttX GPIO framework.
 *
 *   The function creates device nodes under /dev/gpio* that applications
 *   can use to interact with the GPIO pins. The numbering is sequential
 *   across all pin types (inputs first, then outputs, then interrupts).
 *
 *   For the Nucleo-H753ZI board, this typically results in:
 *   - /dev/gpio0 : User button (input)
 *   - /dev/gpio1 : LD1 Green LED (output)
 *   - /dev/gpio2 : LD2 Orange LED (output)
 *   - /dev/gpio3 : LD3 Red LED (output)
 *   - /dev/gpio4 : Custom interrupt pin (interrupt)
 *
 *   Applications can then use standard file operations:
 *   - open("/dev/gpio1", O_RDWR)
 *   - ioctl(fd, GPIOC_WRITE, 1)  // Turn on LED
 *   - ioctl(fd, GPIOC_READ, &value)  // Read current state
 *   - close(fd)
 *
 *   This function is typically called during board initialization, before
 *   applications start running.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Side Effects:
 *   - Configures STM32 GPIO hardware for all defined pins
 *   - Creates device nodes in /dev filesystem
 *   - Initializes all GPIO device structures
 *   - May enable GPIO peripheral clocks (handled by lower-level functions)
 *
 * Notes:
 *   - This function is only compiled if CONFIG_DEV_GPIO is enabled
 *   - Requires board-specific GPIO definitions in the board header file
 *   - Safe to call multiple times (GPIO framework handles re-registration)
 *
 ****************************************************************************/

int stm32_gpio_initialize(void)
{
  int total_pins = 0;
  int pincount = 0;

  gpioinfo("Starting STM32H7 GPIO driver initialization\n");

  /* Initialize each type of GPIO pin in logical order */

  total_pins += stm32_gpio_init_input_pins(&pincount);
  total_pins += stm32_gpio_init_output_pins(&pincount);
  total_pins += stm32_gpio_init_interrupt_pins(&pincount);

  gpioinfo("STM32H7 GPIO driver initialization completed successfully\n");
  gpioinfo("Total GPIO pins initialized: %d\n", total_pins);
  gpioinfo("GPIO devices available: /dev/gpio0 through /dev/gpio%d\n",
           pincount - 1);

  return OK;
}

#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */

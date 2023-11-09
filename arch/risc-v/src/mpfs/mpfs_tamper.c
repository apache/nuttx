/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_tamper.c
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
#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register base-addresses */

#define MPFS_TAMPER_CTRL_BASE 0x4b00c000
#define MPFS_TAMPER_TVS_BASE  0x4b00d000

/* CTRL Register offsets */

#define MPFS_SELECTED_EVENTS_OFFSET         0x00
#define MPFS_STATUS_EVENTS_OFFSET           0x04
#define MPFS_CLEAR_EVENTS_OFFSET            0x08
#define MPFS_ENABLE_EVENTS_OFFSET           0x0c
#define MPFS_ENABLE_TVS_OFFSET              0x10
#define MPFS_RESET_REASON_OFFSET            0x14
#define MPFS_TAMPER_RESPONSE_OFFSET         0x18

/* Selected eventlist */

#define MPFS_SELECTED_EVENTS (MPFS_TAMPER_CTRL_BASE + \
                              MPFS_SELECTED_EVENTS_OFFSET)

#define FLAG_JTAG_ACTIVE                   (1 << 0)
#define FLAG_MESH_ERROR                    (1 << 1)
#define FLAG_CLOCK_MONITOR_GLITCH          (1 << 2)
#define FLAG_CLOCK_MONITOR_FREQ            (1 << 3)
#define FLAG_SECDED                        (1 << 4)
#define FLAG_SCB_BUS_ERROR                 (1 << 5)
#define FLAG_SC_WATCHDOG                   (1 << 6)
#define FLAG_LOCK_ERROR                    (1 << 7)
#define FLAG_DIGEST                        (1 << 8)
#define FLAG_INST_PASSCODE_FAIL            (1 << 9)
#define FLAG_INST_KEY_VALIDATION_FAIL      (1 << 10)
#define FLAG_INST_UNUSED                   (1 << 11)
#define FLAG_BITSTREAM_AUTHENTICATION_FAIL (1 << 12)
#define FLAG_DETECT_LOW_1P0                (1 << 13)
#define FLAG_DETECT_LOW_1P8                (1 << 14)
#define FLAG_DETECT_LOW_2P5                (1 << 15)
#define FLAG_DETECT_HIGH_1P0               (1 << 16)
#define FLAG_DETECT_HIGH_1P8               (1 << 17)
#define FLAG_DETECT_HIGH_2P5               (1 << 18)
#define FLAG_DETECT_TEMPERATURE_LOW        (1 << 19)
#define FLAG_DETECT_TEMPERATURE_HIGH       (1 << 20)

#define FLAGS_ALL                           0x1fffff
#define FLAG_BITS                           21

/* Status eventlist */

#define MPFS_STATUS_EVENTS (MPFS_TAMPER_CTRL_BASE + \
                            MPFS_STATUS_EVENTS_OFFSET)

/* Clear eventlist */

#define MPFS_CLEAR_EVENTS (MPFS_TAMPER_CTRL_BASE + \
                           MPFS_CLEAR_EVENTS_OFFSET)

/* Enable eventlist */

#define MPFS_ENABLE_EVENTS (MPFS_TAMPER_CTRL_BASE + \
                            MPFS_ENABLE_EVENTS_OFFSET)

/* Enable TVS */

#define MPFS_ENABLE_TVS (MPFS_TAMPER_CTRL_BASE +\
                         MPFS_ENABLE_TVS_OFFSET)

#define MPFS_ENABLE_TVS_MONITORING     (1 << 0)
#define MPFS_ENABLE_VOLTAGE_MONITORING (1 << 1)

/* Reset reason */

#define MPFS_RESET_REASON (MPFS_TAMPER_CTRL_BASE + \
                           MPFS_RESET_REASON_OFFSET)

/* Clock and reset */

#define MPFS_SYSREG_SOFT_RESET_CR     (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_SOFT_RESET_CR_OFFSET)
#define MPFS_SYSREG_SUBBLK_CLOCK_CR   (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET)

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/* Mesh system defines */

#define MPFS_MESH_CR     0x200020b0
#define MPFS_MESH_START  (1 << 0)

/* Debug output defines, info promoted to _err for visibility */

#ifdef CONFIG_DEBUG_ERROR
#  define tinfo         _err
#else
#  define tinfo        _none
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_tamper_event_string
 *
 * Description:
 *   Tamper event to string function
 *
 * Input Parameters:
 *   List of events
 *
 * Returned Value:
 *   Readable string
 *
 ****************************************************************************/

static const char *mpfs_tamper_event_string(uint32_t eventlist)
{
  int i;

  static const char *const names[] =
  {
    "JTAG ACTIVE",
    "MESH ERROR",
    "CLOCK MONITOR GLITCH",
    "CLOCK MONITOR FREQ",
    "SECDED",
    "SCB BUS ERROR",
    "SC WATCHDOG",
    "LOCK ERROR",
    "DIGEST",
    "PASSCODE FAIL",
    "KEY VALIDATION FAIL",
    "INST UNUSED",
    "BITSTREAM AUTH FAIL",
    "LOW 1P0",
    "LOW 1P8",
    "LOW 2P5",
    "HIGH 1P0",
    "HIGH 1P8",
    "HIGH 2P5",
    "TEMPERATURE LOW",
    "TEMPERATURE HIGH",
  };

  for (i = 0; i < ARRAY_SIZE(names); i++)
    {
      if ((1 << i) & eventlist)
        {
          break;
        }
    }

  if (i >= ARRAY_SIZE(names))
    {
      return "UNDEFINED";
    }

  return names[i];
}

/****************************************************************************
 * Name: mpfs_tamper_interrupt
 *
 * Description:
 *   Tamper intinfoupt handler
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int mpfs_tamper_interrupt(int irq, void *context, void *arg)
{
  uint32_t eventlist;
  uint32_t sel;
  uint32_t status;
  uint32_t clr;
  uint32_t enabled;
  uint32_t entvs;
  int i;

  sel     = getreg32(MPFS_SELECTED_EVENTS);
  status  = getreg32(MPFS_STATUS_EVENTS);
  clr     = getreg32(MPFS_CLEAR_EVENTS);
  enabled = getreg32(MPFS_ENABLE_EVENTS);
  entvs   = getreg32(MPFS_ENABLE_TVS);

  eventlist = sel;

  if (eventlist)
    {
      tinfo("**********************************************************\n");
      tinfo("Tamper detection has caught out the following event(s):\n");
      for (i = 0; i < FLAG_BITS; i++)
        {
          if ((1 << i) & eventlist)
            {
              tinfo("  <%s>\n", mpfs_tamper_event_string(eventlist));
              eventlist &= ~(1 << i);
            }
        }

      tinfo("Regs: 0x00: 0x%x, 0x04: 0x%x, 0x08: 0x%x, 0x0c: 0x%x,"
            " 0x10: 0x%x\n",
            sel, status, clr, enabled, entvs);
      tinfo("**********************************************************\n");
    }

  putreg32(sel, MPFS_CLEAR_EVENTS);

  return 0;
}

/****************************************************************************
 * Name: mpfs_tamper_tests
 *
 * Description:
 *   Various tamper tests for testing
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef TAMPER_TESTS
extern int mpfs_systemservice_init();
extern int mpfs_sys_get_serial_number(uint8_t *out_sn, uint16_t mb_offset);
extern int mpfs_sys_query_security(uint8_t *out_security_resp,
                                   uint16_t mb_offset);
extern int mpfs_sys_query_design_information(uint8_t *out_security_resp,
                                             uint16_t mb_offset);
extern int mpfs_sys_query_device_certificate(uint8_t *out_security_resp,
                                             uint16_t mb_offset);
extern uint16_t mpfs_sys_unlock_debug_passcode(uint8_t *cmd_data,
                                               uint16_t mb_offset,
                                               uint16_t resp_offset);
extern void test_ecdsa(void);

static void mpfs_tamper_tests(void)
{
  uint8_t buf[1024 + 4];

  mpfs_systemservice_init();

  /* Some of the functions do not exist */

  mpfs_sys_get_serial_number(buf, 0);
  mpfs_sys_query_security(buf, 0);
  mpfs_sys_query_design_information(buf, 0);
  mpfs_sys_query_device_certificate(buf, 0);
  mpfs_sys_unlock_debug_passcode(buf, 0, 0);
  test_ecdsa();
  mpfs_sys_authenticate_iap_image(0);
}
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_tamper_enable
 *
 * Description:
 *   Enables the tamper service
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpfs_tamper_enable(void)
{
  int ret;

  modifyreg32(MPFS_SYSREG_SOFT_RESET_CR,
              SYSREG_SOFT_RESET_CR_FPGA | SYSREG_SOFT_RESET_CR_FIC3,
              0);

  modifyreg32(MPFS_SYSREG_SUBBLK_CLOCK_CR, 0,
              SYSREG_SUBBLK_CLOCK_CR_FIC3);

  tinfo("Enabling Tamper detection - if no FPGA support, will hang here\n");
  tinfo("Tamper reset reason 0x%x\n", getreg32(MPFS_RESET_REASON));

  /* Enable events */

  putreg32(FLAGS_ALL, MPFS_ENABLE_EVENTS);
  putreg32(MPFS_ENABLE_TVS_MONITORING | MPFS_ENABLE_VOLTAGE_MONITORING,
           MPFS_ENABLE_TVS);

  /* Start the mesh system */

  modifyreg32(MPFS_MESH_CR, 0, MPFS_MESH_START);

  ret = irq_attach(MPFS_IRQ_FABRIC_F2H_10, mpfs_tamper_interrupt, NULL);

  if (ret == 0)
    {
      up_enable_irq(MPFS_IRQ_FABRIC_F2H_10);
    }
  else
    {
      tinfo("Tamper IRQ attach failed");
    }

#ifdef TAMPER_TESTS
  mpfs_tamper_tests();
#endif
}

/****************************************************************************
 * Name: mpfs_tamper_disable
 *
 * Description:
 *   Disables tamper service
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpfs_tamper_disable(void)
{
  modifyreg32(MPFS_MESH_CR, MPFS_MESH_START, 0);

  up_disable_irq(MPFS_IRQ_FABRIC_F2H_10);
  irq_detach(MPFS_IRQ_FABRIC_F2H_10);

  putreg32(0x00, MPFS_ENABLE_EVENTS);
  putreg32(0, MPFS_ENABLE_TVS);
}

#include "ch.h"
#include "hal.h"

#include "dbus.h"
#include "sbus.h"

#include "system_error.h"

static system_error_t system_error = 0;
static systime_t system_error_start_time;
static systime_t system_warning_start_time;

/*
 *====================LED INDICATOR========
 *   o-o-o-o-o-o-o-         DBUS Not CONNECTED
 *   o------o------         DBUS RC locked
 *   o-o----o-o----         DBUS RC unlocked
 *   oooooooooooooo         System failed
 *   --------------         System failed
 *   RED led blinking       ERROR occured, requires a reboot
 *   YELLOW led blinking    Warning occured,
 *   GREED led blinking     System normal
 *=========================================
 */

void system_setErrorFlag(void)
{
  system_error |= SYSTEM_ERROR;
}

void system_setWarningFlag(void)
{
  system_error |= SYSTEM_WARNING;
}

void system_setTempErrorFlag(void)
{
  system_error_start_time = chVTGetSystemTimeX();
  system_error |= SYSTEM_TEMP_ERROR;
}

void system_setTempWarningFlag(void)
{
  system_warning_start_time = chVTGetSystemTimeX();
  system_error |= SYSTEM_TEMP_WARNING;
}

void system_clearWarningFlag(void)
{
  system_error &= ~(SYSTEM_TEMP_WARNING | SYSTEM_WARNING);
}

static THD_WORKING_AREA(system_error_wa, 128);
static THD_FUNCTION(system_error_thd, p)
{
  (void) p;
  chRegSetThreadName("System status indicator");

  LEDG_OFF();
  LEDR_OFF();

  rc_state_t   rc_state_slave;
  SBUS_state_t rc_state_master;
  bool led_on; //Control the toggling of LED

  uint32_t count = 0;
  while(true)
  {
    //Control the flashing of green LED // Shift to Error.c
    rc_state_master = SBUS_getState();
    rc_state_slave  = RC_getState();
    if(!(count % 5))
    {
      uint32_t blink_count = count / 5;

      if(!(blink_count % 15))
        led_on = false;

      if((!rc_state_slave && rc_state_master != SBUS_CONNECTED)      ||
         (rc_state_master == SBUS_CONNECTED && !rc_state_slave &&
           blink_count % 15 < 4)                                     ||
         (rc_state_slave  && rc_state_master != SBUS_CONNECTED &&
           blink_count % 15 < 2)                                     ||
         (rc_state_master == SBUS_CONNECTED && rc_state_slave &&
           blink_count % 15 < 6)
        )
        {
          led_on = !led_on;
          if(!(system_error & (SYSTEM_ERROR | SYSTEM_TEMP_ERROR)))
            led_on ? LEDG_ON() : LEDG_OFF();
          else
            LEDG_OFF();

          if(system_error)
            led_on ? LEDR_ON() : LEDR_OFF();
          else
            LEDR_OFF();
        }
    }

    count++;
    if(chVTGetSystemTimeX() > system_warning_start_time + S2ST(SYSTEM_TEMP_WARNING_DURATION))
      system_error &= ~(SYSTEM_TEMP_WARNING);
    if(chVTGetSystemTimeX() > system_error_start_time + S2ST(SYSTEM_TEMP_WARNING_DURATION))
      system_error &= ~(SYSTEM_TEMP_ERROR);

    chThdSleepMilliseconds(25);
  }
}

void system_error_init(void)
{
  chThdCreateStatic(system_error_wa, sizeof(system_error_wa),
                    NORMALPRIO - 7, system_error_thd, NULL);
}

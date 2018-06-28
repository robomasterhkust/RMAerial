#ifndef _MAIN_H_
#define _MAIN_H_

#include "ch.h"
#include "hal.h"

#include "math_misc.h"

#include "usbcfg.h"
#include "flash.h"
#include "chprintf.h"
#include "system_error.h"

#include "canBusProcess.h"
#include "dbus.h"
#include "params.h"
#include "sdlog.h"
#include "osdk_comm.h"
#include "osdk_drone_cmd.h"

#include "attitude.h"
#include "calibrate_sensor.h"

#include "gimbal.h"
#include "shoot.h"
#include "feeder.h"

#include "exti.h"

#define LASER_ON()  (palSetPad(GPIOA, GPIOA_PIN0))
#define LASER_OFF() (palClearPad(GPIOA, GPIOA_PIN0))

typedef enum {
  INIT_DUMMY = 0,
  INIT_SEQUENCE_3_RETURN_1 = 1,
  INIT_SEQUENCE_3_RETURN_2 = 2,
  INIT_SEQUENCE_3_RETURN_3 = 4,
  INIT_ATTITUDE_COMPLETE = 16,
  INIT_COMPLETE = 32
} system_init_state_t;

#ifdef __cplusplus
extern "C" {
#endif

void shellStart(void);

#ifdef __cplusplus
}
#endif

bool power_check(void);
bool power_failure(void);

#endif

#ifndef _MAIN_H_
#define _MAIN_H_

#include "ch.h"
#include "hal.h"

#include "math_misc.h"

#include "usbcfg.h"
#include "flash.h"
#include "chprintf.h"

#include "canBusProcess.h"
#include "dbus.h"
#include "params.h"
#include "sdlog.h"
#include "osdk_comm.h"
#include "osdk_drone_cmd.h"

#include "mpu6500.h"
#include "ist8310.h"
#include "attitude.h"
#include "calibrate_sensor.h"

#include "gimbal.h"
#include "shoot.h"
#include "feeder.h"

#include "exti.h"
#include "imu_temp.h"

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

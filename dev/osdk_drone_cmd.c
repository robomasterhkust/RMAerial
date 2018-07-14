#include "ch.h"
#include "hal.h"

#include "sbus.h"
#include "dbus.h"
#include "osdk_comm.h"
#include "osdk_drone_cmd.h"
#include "math_misc.h"

#include <string.h>

static bool    airborne = false;
static float   init_yaw;
static int16_t flight_mode = OSDK_RC_MODE_DUMMY;

static SBUS_t* rc_master;

void droneCmd_OSDK_control(const uint8_t enable)
{
  uint8_t cmd_val = (enable == ENABLE ? 1 : 0);

  //Send twice as instructed by DJI OSDK manual
  osdk_StartTX_NoACK(&cmd_val, 1, OSDK_CTRL_CMD_SET,
    OSDK_OBTAIN_CTRL_ID, OSDK_TX_WAIT);

  chThdSleepMilliseconds(100);

  osdk_StartTX_NoACK(&cmd_val, 1, OSDK_CTRL_CMD_SET,
    OSDK_OBTAIN_CTRL_ID, OSDK_TX_WAIT);
}

void droneCmd_Flight_control(const uint8_t ctrl_mode,
                             const float x,
                             const float y,
                             const float z,
                             const float yaw)
{
  CtrlData_t ctrl;

  ctrl.flag = ctrl_mode;
  ctrl.x = x;
  ctrl.y = y;
  ctrl.z = z;
  ctrl.yaw = yaw;

  osdk_StartTX_NoACK(&ctrl, sizeof(CtrlData_t), OSDK_CTRL_CMD_SET,
    OSDK_MOVEMENT_CTRL_ID, OSDK_TX_WAIT);
}

/*
 *  @NOTE: This function will arm the drone motor autonomously
 *         To avoid serious accident, NEVER USE it in lab,
 *         use ONLY in competition field, simulator, or secured test fields
 */
void droneCmd_armMotor_control(const uint8_t enable)
{
  osdkComm_t* comm = osdkComm_get();
  if(comm->rxchn_state != OSDK_RXCHN_STATE_STABLE)
    return;

  //===================================================================
  airborne = true; //TODO: change this to update with osdk flight data;
  //===================================================================

  uint8_t cmd_val = (enable == ENABLE ? 1 : 0);

  //Send twice as instructed by DJI OSDK manual
  osdk_StartTX_NoACK(&cmd_val, 1, OSDK_CTRL_CMD_SET,
    OSDK_ARM_CMD_ID, OSDK_TX_WAIT);
}


static THD_WORKING_AREA(drone_cmd_wa, 1024);
static THD_FUNCTION(drone_cmd, p)
{
  (void)p;
  chRegSetThreadName("DJI drone CMD host");

  RC_Ctl_t* rc_slave = RC_get();

  osdkComm_t* comm = osdkComm_get();
  while(comm->rxchn_state != OSDK_RXCHN_STATE_STABLE)
    chThdSleepMilliseconds(100);

  chThdSleepMilliseconds(500);

  osdk_activate(OSDK_APP_ID);
  osdk_attitude_subscribe();
  osdk_velocity* vel = osdk_velocity_subscribe();

  uint32_t guidance_online_counter = 0;

  while(!airborne) //Wait for aircraft to take off;
  {
    if(
       rc_master->ch1 > SBUS_CH_VALUE_MAX - 5 &&
       rc_master->ch2 < SBUS_CH_VALUE_MIN + 5 &&
       rc_master->ch3 < SBUS_CH_VALUE_MIN + 5 &&
       rc_master->ch4 < SBUS_CH_VALUE_MIN + 5
      )
      airborne = true;
    chThdSleepMilliseconds(10);
  }

  init_yaw = osdk_attitude_get_yaw();
  float sine = sinf(init_yaw),
        cosine = cosf(init_yaw);

  const uint8_t flight_mode_A =
          OSDK_FLIGHT_MODE_HORI_ATTI |
          OSDK_FLIGHT_MODE_VERT_VEL  |
          OSDK_FLIGHT_MODE_YAW_RATE  |
          OSDK_FLIGHT_MODE_GND_FRAME |
          OSDK_FLIGHT_MODE_NON_STABLE; //W/O guidance sensor

  const uint8_t flight_mode_P =
          OSDK_FLIGHT_MODE_HORI_VEL  |
          OSDK_FLIGHT_MODE_VERT_VEL  |
          OSDK_FLIGHT_MODE_YAW_RATE  |
          OSDK_FLIGHT_MODE_GND_FRAME |
          OSDK_FLIGHT_MODE_NON_STABLE; //With guidance sensor or GPS

  uint8_t ctrl_mode = flight_mode_A;

  systime_t tick = chVTGetSystemTimeX();
  while(true)
  {
    tick += US2ST(1e6/DRONE_CMD_FREQ);
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }


    if(flight_mode == OSDK_RC_MODE_P_SDK)
    {
      float x, y, z, yaw;

      if(vel->info.health == 1) //Guidance data valid
        guidance_online_counter++;
      else
        guidance_online_counter = 0;

      if(guidance_online_counter > 3)
        ctrl_mode = flight_mode_P;
      else
        ctrl_mode = flight_mode_A;

      if(ctrl_mode == flight_mode_A)
      {
        x = mapInput(rc_master->ch1,    SBUS_CH_VALUE_MIN, SBUS_CH_VALUE_MAX,
          USER_CTRL_HORI_ATTI_MIN, USER_CTRL_HORI_ATTI_MAX);
        y = -mapInput(rc_master->ch2,  SBUS_CH_VALUE_MIN, SBUS_CH_VALUE_MAX,
          USER_CTRL_HORI_ATTI_MIN, USER_CTRL_HORI_ATTI_MAX);
      }
      else if(ctrl_mode == flight_mode_P)
      {
        y = mapInput(rc_master->ch1,    SBUS_CH_VALUE_MIN, SBUS_CH_VALUE_MAX,
          USER_CTRL_HORI_VEL_MIN,  USER_CTRL_HORI_VEL_MAX);
        x = mapInput(rc_master->ch2,  SBUS_CH_VALUE_MIN, SBUS_CH_VALUE_MAX,
          USER_CTRL_HORI_VEL_MIN,  USER_CTRL_HORI_VEL_MAX);
      }

      z = mapInput(rc_master->ch3, SBUS_CH_VALUE_MIN, SBUS_CH_VALUE_MAX,
        USER_CTRL_VERT_VEL_MIN, USER_CTRL_VERT_VEL_MAX);

      if(RC_getState() == RC_UNLOCKED)
        yaw = mapInput(rc_slave->rc.channel2,  RC_CH_VALUE_MIN, RC_CH_VALUE_MAX,
          USER_CTRL_YAW_RATE_MIN, USER_CTRL_YAW_RATE_MAX);
      else
        yaw = mapInput(rc_master->ch4,  SBUS_CH_VALUE_MIN, SBUS_CH_VALUE_MAX,
          USER_CTRL_YAW_RATE_MIN, USER_CTRL_YAW_RATE_MAX);

      //PID tracking controller for gimbal heading
      float yaw_feedback = 0.0f;
      yaw += yaw_feedback;

      float y_out = y * cosine + x * sine,
            x_out = x * cosine - y * sine;

      droneCmd_Flight_control(ctrl_mode, x_out, y_out, z, yaw);
    }
  }
}

static THD_WORKING_AREA(dji_sdk_wa, 128);
static THD_FUNCTION(dji_sdk, p)
{
  (void)p;
  chRegSetThreadName("DJI osdk control");

  osdkComm_t* comm = osdkComm_get();
  while(comm->rxchn_state != OSDK_RXCHN_STATE_STABLE)
    chThdSleepMilliseconds(100);

  chThdSleepSeconds(2);

  while(true)
  {
    if(flight_mode != rc_master->ch6)
    {
      if(rc_master->ch6 == OSDK_RC_MODE_P_SDK)
        droneCmd_OSDK_control(ENABLE);
      else
        droneCmd_OSDK_control(DISABLE);
    }

    flight_mode = rc_master->ch6;
    chThdSleepMilliseconds(20);
  }
}

void droneCmd_init(void)
{
  if(*UART_OSDK.state != UART_READY)
    return;

  rc_master = SBUS_get();

  chThdCreateStatic(dji_sdk_wa, sizeof(dji_sdk_wa),
    NORMALPRIO + 4, dji_sdk ,NULL);
  chThdCreateStatic(drone_cmd_wa, sizeof(drone_cmd_wa),
    NORMALPRIO - 4, drone_cmd ,NULL);
}

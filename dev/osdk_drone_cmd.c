#include "ch.h"
#include "hal.h"

#include "dbus.h"
#include "osdk_comm.h"
#include "osdk_drone_cmd.h"
#include "math_misc.h"

#include <string.h>

static bool    airborne = false;
static float   init_yaw;
static int16_t flight_mode = OSDK_RC_MODE_DUMMY;

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
  init_yaw = osdk_attitude_get_yaw();
  //===================================================================

  uint8_t cmd_val = (enable == ENABLE ? 1 : 0);

  //Send twice as instructed by DJI OSDK manual
  osdk_StartTX_NoACK(&cmd_val, 1, OSDK_CTRL_CMD_SET,
    OSDK_ARM_CMD_ID, OSDK_TX_WAIT);
}

static void droneCmd_virtualRC_reset(VirtualRC_Data* vRC)
{
  vRC->roll     = RC_CH_VALUE_OFFSET;
  vRC->pitch    = RC_CH_VALUE_OFFSET;
  vRC->throttle = RC_CH_VALUE_OFFSET;
  vRC->yaw      = RC_CH_VALUE_OFFSET;
  vRC->gear     = 1324;
  vRC->mode     = 1552;
}

static void droneCmd_virtualRC_init(VirtualRC_Data* vRC)
{
  droneCmd_virtualRC_reset(vRC);

  uint8_t i;
  uint32_t* channel = (uint32_t*)(vRC);
  for(i = 7; i < 16; i++)
    channel[i] = RC_CH_VALUE_OFFSET;
}

static void droneCmd_virtualRC_cmd(uint8_t enable)
{
  VirtualRC_Setting setting;

  setting.reserved = 0;
  setting.cutoff = 1;
  setting.enable = (enable == ENABLE ? 1 : 0);

  osdk_StartTX_NoACK((uint8_t*)&setting, 1,
    OSDK_VIRTUALRC_SET, OSDK_VIRTUALRC_REQ_ID, OSDK_TX_WAIT);
}

static void droneCmd_virtualRC_setValue(RC_Ctl_t* const rc1, RC_Ctl_t* const rc2,
                              VirtualRC_Data* const vRC)
{
  if(RC_getState() == RC_UNLOCKED)
  {
    switch(rc1->rc.s1)
    {
      case RC_S_UP:
        vRC->gear = 1684; //CL on

        float yaw_diff =  osdk_attitude_get_yaw() - init_yaw;
        float sine   = sinf(yaw_diff),
              cosine = cosf(yaw_diff);
        float roll_in  = (float)rc1->rc.channel0 - (float)RC_CH_VALUE_OFFSET,
              pitch_in = (float)rc1->rc.channel1 - (float)RC_CH_VALUE_OFFSET;

        vRC->roll  =
          (uint32_t)(roll_in * cosine - pitch_in * sine + (float)RC_CH_VALUE_OFFSET);
        vRC->pitch =
          (uint32_t)(pitch_in * cosine + roll_in * sine + (float)RC_CH_VALUE_OFFSET);

        vRC->throttle = (uint32_t)(rc1->rc.channel3);
        vRC->yaw      = (uint32_t)(rc1->rc.channel2);
        break;
      case RC_S_DOWN: //Takeoff, record the initial YAW angle for CL mode
        init_yaw = osdk_attitude_get_yaw();
      case RC_S_MIDDLE:
        vRC->gear = 1324; //CL off
        vRC->roll     = (uint32_t)(rc1->rc.channel0);
        vRC->pitch    = (uint32_t)(rc1->rc.channel1);
        vRC->throttle = (uint32_t)(rc1->rc.channel3);
        vRC->yaw      = (uint32_t)(rc1->rc.channel2);
        break;
    }
    switch(rc1->rc.s2)
    {
      case RC_S_UP:
        vRC->mode = 1552; //P mode
        break;
      case RC_S_MIDDLE:
        vRC->mode = 1024; //A mode
        break;
      case RC_S_DOWN:
        vRC->mode = 496;  //F mode
        break;
    }
  }
  else
    droneCmd_virtualRC_reset(vRC);
}

static THD_WORKING_AREA(drone_cmd_wa, 1024);
static THD_FUNCTION(drone_cmd, p)
{
  (void)p;
  chRegSetThreadName("DJI drone CMD host");

  RC_Ctl_t* rc1 = RC_get();
  uint8_t rc_connected = false;

  osdkComm_t* comm = osdkComm_get();
  while(comm->rxchn_state != OSDK_RXCHN_STATE_STABLE)
    chThdSleepMilliseconds(100);

  chThdSleepMilliseconds(500);

  osdk_activate(OSDK_APP_ID);

  osdk_attitude_subscribe();
  osdk_RC* rc = osdk_RC_subscribe();

  WAIT_TAKEOFF:
  while(!airborne) //Wait for aircraft to take off;
    chThdSleepMilliseconds(100);

  float sine = sinf(init_yaw),
        cosine = cosf(init_yaw);
/*
  static VirtualRC_Data vRC;
  droneCmd_virtualRC_init(&vRC);

  while(!chThdShouldTerminateX())
  {
    tick += US2ST(1e6/DRONE_CMD_FREQ);
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    droneCmd_virtualRC_setValue(rc1, NULL, &vRC);
    if(RC_getState() == RC_UNLOCKED)
    {
      if(!rc_connected)
      {
        droneCmd_virtualRC_cmd(ENABLE);
        rc_connected = true;
      }
      osdk_StartTX_NoACK((uint8_t*)&vRC, sizeof(VirtualRC_Data),
        OSDK_VIRTUALRC_SET, OSDK_VIRTUALRC_DATA_ID, OSDK_TX_WAIT);
    }
    else if(rc_connected)
    {
      osdk_StartTX_NoACK((uint8_t*)&vRC, sizeof(VirtualRC_Data),
        OSDK_VIRTUALRC_SET, OSDK_VIRTUALRC_DATA_ID, OSDK_TX_WAIT);

      droneCmd_virtualRC_cmd(DISABLE);
      rc_connected = false;
    }
  }*/
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

  uint8_t ctrl_mode = flight_mode_P;

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

    /*TODO: Do something*/
    if(flight_mode == OSDK_RC_MODE_P_SDK)
    {
      float x, y, z, yaw;

      if(ctrl_mode == flight_mode_A)
      {
        x = mapInput(rc->pitch,    OSDK_RC_MIN, OSDK_RC_MAX,
          OSDK_CTRL_HORI_ATTI_MIN, OSDK_CTRL_HORI_ATTI_MAX);
        y = mapInput(rc->roll,  OSDK_RC_MIN, OSDK_RC_MAX,
          OSDK_CTRL_HORI_ATTI_MIN, OSDK_CTRL_HORI_ATTI_MAX);
      }
      else if(ctrl_mode == flight_mode_P)
      {
        x = mapInput(rc->pitch,    OSDK_RC_MIN, OSDK_RC_MAX,
          OSDK_CTRL_HORI_VEL_MIN,  OSDK_CTRL_HORI_VEL_MAX);
        y = mapInput(rc->roll,  OSDK_RC_MIN, OSDK_RC_MAX,
          OSDK_CTRL_HORI_VEL_MIN,  OSDK_CTRL_HORI_VEL_MAX);
      }

      z = mapInput(rc->throttle, OSDK_RC_MIN, OSDK_RC_MAX,
        OSDK_CTRL_VERT_VEL_MIN, OSDK_CTRL_VERT_VEL_MAX);
      yaw = mapInput(rc->yaw,    OSDK_RC_MIN, OSDK_RC_MAX,
        OSDK_CTRL_YAW_RATE_MIN, OSDK_CTRL_YAW_RATE_MAX);

      //Reversed stick direction to test

      float y_out = y * cosine + x * sine,
            x_out = x * cosine - y * sine;

      droneCmd_Flight_control(ctrl_mode, x_out, y_out, z, yaw);

      if(false) //The drone has landed, waiting to take_off again
        goto WAIT_TAKEOFF;
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

  osdk_RC* rc = osdk_RC_subscribe();

  chThdSleepSeconds(1);

  while(true)
  {
    if(flight_mode != rc->mode)
    {
      if(rc->mode == OSDK_RC_MODE_P_SDK)
        droneCmd_OSDK_control(ENABLE);
      else
        droneCmd_OSDK_control(DISABLE);
    }

    flight_mode = rc->mode;
    chThdSleepMilliseconds(20);
  }
}

void droneCmd_init(void)
{
  if(*UART_OSDK.state != UART_READY)
    return;

  chThdCreateStatic(dji_sdk_wa, sizeof(dji_sdk_wa),
    NORMALPRIO + 4, dji_sdk ,NULL);
  chThdCreateStatic(drone_cmd_wa, sizeof(drone_cmd_wa),
    NORMALPRIO - 4, drone_cmd ,NULL);
}

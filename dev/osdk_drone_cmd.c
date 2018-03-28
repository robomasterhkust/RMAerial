#include "ch.h"
#include "hal.h"

#include "dbus.h"
#include "osdk_comm.h"
#include "osdk_drone_cmd.h"
#include <string.h>

const char activation_string[] = "12345678901234567890123456789012";

uint16_t droneCmd_activate(uint32_t app_id)
{
  uint8_t activation_seq[44];
  *((uint32_t*)activation_seq)       = app_id;
  *((uint32_t*)(activation_seq + 4)) = (uint32_t)2;
  *((uint32_t*)(activation_seq + 8)) = OSDK_ACTIVATION_KEY;
  memcpy(activation_seq + 12, activation_string, 32);

  uint16_t result = osdk_StartTX_ACK(activation_seq, 44,
    OSDK_ACTIVATION_SET, OSDK_ACTIVATION_ID, MS2ST(1000));
  return result;
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

static void droneCmd_virtualRC_setValue(RC_Ctl_t* const rc,
                              VirtualRC_Data* const vRC)
{
  if(rc->state == RC_STATE_CONNECTED)
  {
    vRC->roll     = (uint32_t)(rc->rc.channel0);
    vRC->pitch    = (uint32_t)(rc->rc.channel1);
    vRC->throttle = (uint32_t)(rc->rc.channel3);
    vRC->yaw      = (uint32_t)(rc->rc.channel2);
    switch(rc->rc.s1)
    {
      case RC_S_UP:
        vRC->gear = 1684; //Down
        break;
      case RC_S_MIDDLE:
        vRC->gear = 1324; //Down
        break;
      case RC_S_DOWN:
        vRC->gear = 1324; //Down
        break;
    }
    switch(rc->rc.s2)
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

  systime_t tick = chVTGetSystemTimeX();

  RC_Ctl_t* rc = RC_get();
  uint8_t rc_connected = false;

  osdkComm_t* comm = osdkComm_get();
  while(comm->rxchn_state != OSDK_RXCHN_STATE_STABLE)
    chThdSleepMilliseconds(100);
  osdk_attitude_subscribe();

  droneCmd_activate(OSDK_APP_ID);

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

    droneCmd_virtualRC_setValue(rc, &vRC);
    if(rc->state == RC_STATE_CONNECTED)
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
  }
}

void droneCmd_init(void)
{
  if(*UART_OSDK.state != UART_READY)
    return;

  chThdCreateStatic(drone_cmd_wa, sizeof(drone_cmd_wa), NORMALPRIO + 4, drone_cmd ,NULL);
}

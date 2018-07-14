#include "keyboard.h"
#include "canBusProcess.h"
#include "ch.h"
#include "hal.h"
#include "dbus.h"
int bitmap[15] = {};
kb_state_e keyboard[15] = {};
/* mouse button long press time */
#define LONG_PRESS_TIME  1000  //ms
/* key acceleration time */
#define slide_ratio 1
#define Up_ratio 0.5
#define Normal_ratio 0.4
#define Down_ratio 0.2
RC_Ctl_t* dbusP;
kb_ctrl_t km;

void keyboard_to_bitmap(RC_Ctl_t* pRC){
  uint8_t i = 0;
  //uint32_t n = RC_get()->keyboard.key_code;
  uint32_t n = pRC->keyboard.key_code;

  int j;
  for(j=0 ; j< 15; j++){
    bitmap[i] = n % 2;
    n = n/2;
    i++;
  }

}
/*
bool keyboard_enable(Gimbal_Send_Dbus_canStruct* pRC){
  return (pRC->s1 == MI);
  //return RC_get()->rc.s2 == UP;
}*/

void keyboard_reset(){
  km.vx = 0;
  km.vy = 0;
  km.vw = 0;
  km.twist_ctrl = 0;
}

static void move_direction_ctrl(uint8_t forward, uint8_t back,
                                uint8_t left,    uint8_t right)
{
  //add ramp
  if (forward)
  {
    if(left || right){
      km.vy = km.y_spd_limit/1.414;
    }
    else{
      km.vy = km.y_spd_limit;
    }
  }
  else if (back)
  {
    if(left || right){
      km.vy = -km.y_spd_limit/1.414;
    }
    else{
      km.vy = -km.y_spd_limit;
    }
  }
  else
  {
    km.vy = 0;
  }

  if (left)
  {
    if(forward || back){
      km.vx = -km.x_spd_limit/1.414;
    }
    else{
      km.vx = -km.x_spd_limit;
    }
  }
  else if (right)
  {
    if(forward || back){
      km.vx = km.x_spd_limit/1.414;
    }
    else{
      km.vx = km.x_spd_limit;
    }
  }
  else
  {
    km.vx = 0;
  }

  if (forward || back || left || right)
    km.twist_ctrl = 0;
}


static THD_WORKING_AREA(keyboard_decode_wa, 512);
static THD_FUNCTION(keyboard_decode, p){
    (void) p;
    chRegSetThreadName("keyboard decode");
    uint32_t tick = chVTGetSystemTimeX();
    while(!chThdShouldTerminateX())
    {
      tick += US2ST(KEYBOARD_UPDATE_PERIOD_US);
      if(tick > chVTGetSystemTimeX()){
        chThdSleepUntil(tick);
      }
      keyboard_to_bitmap(dbusP);
    }
}

void keyboardInit(){
  dbusP = RC_get();
  chThdCreateStatic(keyboard_decode_wa, sizeof(keyboard_decode_wa),
                   NORMALPRIO - 5, keyboard_decode, NULL);

}


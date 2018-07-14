#include "ch.h"
#include "hal.h"

#include "params.h"
#include "canBusProcess.h"
#include "dbus.h"

#include "math_misc.h"

#include "feeder.h"

static int16_t FEEDER_SPEED_SP_RPM  = 0;
#define FEEDER_TURNBACK_ANGLE   360.0f / FEEDER_BULLET_PER_TURN     //165.0f;

static int16_t        feeder_output;
static uint8_t        level;
static uint8_t        feeder_fire_mode = FEEDER_AUTO; //User selection of firing mode
static feeder_mode_t  feeder_mode = FEEDER_STOP;
static float          feeder_brakePos = 0.0f;
static systime_t      feeder_start_time;
static systime_t      bullet_out_time;
static systime_t      feeder_stop_time;
static thread_reference_t rune_singleShot_thread = NULL;

static uint8_t feeder_error_flag;

#define FEEDER_BOOST_SETSPEED_SINGLE    20  * FEEDER_GEAR * 60 / FEEDER_BULLET_PER_TURN
#define FEEDER_BOOST_SETSPEED_AUTO      30  * FEEDER_GEAR * 60 / FEEDER_BULLET_PER_TURN
#define FEEDER_TEST_SETSPEED             3  * FEEDER_GEAR * 60 / FEEDER_BULLET_PER_TURN
#define FEEDER_BOOST_PERIOD_MS          30

static float bullet_delay;

ChassisEncoder_canStruct*   feeder_encode;
RC_Ctl_t*                   p_dbus;
BarrelStatus_canStruct*     barrel_info;
static lpfilterStruct lp_spd_feeder;

pid_struct  vel_pid /*= {0, 0, 0, 0}*/;
pid_struct  pos_pid /*= {5.0, 0, 0, 0, 0}*/;
pid_struct  rest_pid /*= {0.45, 0, 0, 0, 0}*/;

uint16_t error_count = 0;

static uint32_t bulletCount      = 0;
static uint32_t bulletCount_stop = 0;

bool feeder_get_error(void)
{
  return feeder_error_flag;
}

void feeder_clear_error(void)
{
  feeder_error_flag = false;
}

int16_t feeder_canUpdate(void)
{
  #if (FEEDER_CAN_EID == 0x1FF)
    return feeder_output;
  #elif (FEEDER_CAN_EID == 0x200)
    can_motorSetCurrent(FEEDER_CAN, FEEDER_CAN_EID,\
        feeder_output, 0, 0, 0);
  #endif
}

float feeder_getDelay(void)
{
  return bullet_delay;
}

static void feeder_brake(void)
{
  feeder_brakePos = (float)feeder_encode->total_ecd;
  pos_pid.inte = 0; //reset pid integrator
  feeder_stop_time = chVTGetSystemTimeX();
}

void feeder_bulletOut(void)
{
  if(chVTGetSystemTimeX() > bullet_out_time + MS2ST(10))
  {
    bullet_out_time = chVTGetSystemTimeX();
    LEDR_TOGGLE();

    #ifdef FEEDER_USE_BOOST
      if(feeder_mode == FEEDER_BOOST)
      {
        if(rune_singleShot_thread == NULL) //Disable mode selection during rune shooting
          feeder_mode = feeder_fire_mode;// TODO: select fire mode using keyboard input
        else
          feeder_mode = FEEDER_SINGLE;

        vel_pid.inte = 0; //reset pid integrator
        bullet_delay = ST2US(bullet_out_time - feeder_start_time)/1e3f;
      }
    #endif

    if(feeder_mode == FEEDER_SINGLE)
    {
      if(rune_singleShot_thread != NULL)
      {
        chThdResumeI(&rune_singleShot_thread, MSG_OK);
        rune_singleShot_thread = NULL;
      }
      feeder_brake();
      feeder_mode = FEEDER_FINISHED;
    }
  }
}

void feeder_singleShot(void)
{
  #ifdef FEEDER_USE_BOOST
    feeder_mode = FEEDER_BOOST;
  #else
    feeder_mode = FEEDER_SINGLE;
  #endif

  chSysLock();
  chThdSuspendS(&rune_singleShot_thread);
  chSysUnlock();
}

static void feeder_rest(void)
{
    int16_t current_speed = lpfilter_apply(&lp_spd_feeder, feeder_encode->raw_speed);
    float error = - (float) current_speed;
    rest_pid.inte += error * rest_pid.ki;
    rest_pid.inte = rest_pid.inte > rest_pid.inte_max?  rest_pid.inte_max:rest_pid.inte;
    rest_pid.inte = rest_pid.inte <-rest_pid.inte_max? -rest_pid.inte_max:rest_pid.inte;

    feeder_output = rest_pid.kp * error + rest_pid.inte;
    feeder_output = feeder_output > 4000?  4000:feeder_output;
    feeder_output = feeder_output <-4000? -4000:feeder_output;

    feeder_canUpdate();
}

static int16_t feeder_controlVel(const float target, const float output_max){
    static float last_error;
    static float current_error;

    last_error = current_error;
    int16_t current_speed = feeder_encode->raw_speed;
    current_speed = lpfilter_apply(&lp_spd_feeder, current_speed);
    current_error = target - (float) current_speed;
    vel_pid.inte += current_error * vel_pid.ki;
    vel_pid.inte = vel_pid.inte > vel_pid.inte_max?  vel_pid.inte_max:vel_pid.inte;
    vel_pid.inte = vel_pid.inte <-vel_pid.inte_max? -vel_pid.inte_max:vel_pid.inte;

    float output = vel_pid.kp * current_error + vel_pid.inte + vel_pid.kd * (current_error - last_error);
    output = output > output_max?  output_max:output;
    output = output <-output_max? -output_max:output;

    return (int16_t) output;

}

static int16_t feeder_controlPos(const float target, const float output_max){

    float error = target - (float)feeder_encode->total_ecd;

    pos_pid.inte += error * pos_pid.ki;
    pos_pid.inte = pos_pid.inte > pos_pid.inte_max?  pos_pid.inte_max:pos_pid.inte;
    pos_pid.inte = pos_pid.inte <-pos_pid.inte_max? -pos_pid.inte_max:pos_pid.inte;

    float speed_sp = pos_pid.kp * error +
                     pos_pid.inte -
                     pos_pid.kd * feeder_encode->raw_speed;

    return feeder_controlVel(speed_sp, output_max);
}

static void feeder_func(){
    feeder_output = 0.0f;
    switch (feeder_mode){
        case FEEDER_FINISHED:
        case FEEDER_OVERHEAT:
        case FEEDER_STOP:
            if(chVTGetSystemTimeX() > feeder_stop_time + MS2ST(500))
              feeder_rest();
            else
            {
              feeder_output = feeder_controlPos(feeder_brakePos, FEEDER_OUTPUT_MAX);
              feeder_canUpdate();
            }
            break;
        case FEEDER_SINGLE:
            #ifndef FEEDER_USE_BOOST
              if(chVTGetSystemTimeX() - feeder_start_time > MS2ST(1000))
              {
                feeder_mode = FEEDER_FINISHED;
                feeder_brake();
              }
            #endif
        case FEEDER_AUTO:
            //error detecting
            if (
                 state_count((feeder_encode->raw_speed < 30) &&
                             (feeder_encode->raw_speed > -30),
                 FEEDER_ERROR_COUNT, &error_count)
               )
            {
              float error_angle_sp = feeder_encode->total_ecd -
                FEEDER_TURNBACK_ANGLE / 360.0f * FEEDER_GEAR * 8192;
              systime_t error_start_time = chVTGetSystemTime();
              while (chVTIsSystemTimeWithin(error_start_time, (error_start_time + MS2ST(200))) )
              {
                feeder_output = feeder_controlPos(error_angle_sp, FEEDER_OUTPUT_MAX_BACK);
                feeder_canUpdate();
                chThdSleepMilliseconds(1);
              }
            }

            feeder_output = feeder_controlVel(FEEDER_SPEED_SP_RPM, FEEDER_OUTPUT_MAX);
            feeder_canUpdate();

            break;
        #ifdef FEEDER_USE_BOOST
          case FEEDER_BOOST:
            if(chVTGetSystemTimeX() - feeder_start_time > MS2ST(1000)) //No bullet, exit
            {
              feeder_mode = FEEDER_FINISHED;
              feeder_brake();
            }
            else if(chVTGetSystemTimeX() - feeder_start_time > MS2ST(FEEDER_BOOST_PERIOD_MS))
            {
              if(feeder_fire_mode == FEEDER_SINGLE)
                FEEDER_SPEED_SP_RPM = FEEDER_BOOST_SETSPEED_SINGLE;
              else
                FEEDER_SPEED_SP_RPM = FEEDER_BOOST_SETSPEED_AUTO;
              feeder_output = feeder_controlVel(FEEDER_SPEED_SP_RPM, FEEDER_OUTPUT_MAX);
            }
            else
              feeder_output = FEEDER_BOOST_POWER;

            feeder_canUpdate();
            if(
                   state_count((feeder_encode->raw_speed < 30) &&
                               (feeder_encode->raw_speed > -30),
                   FEEDER_ERROR_COUNT, &error_count)
              ) //Bullet stuck
            {
              float error_angle_sp = feeder_encode->total_ecd -
                FEEDER_TURNBACK_ANGLE / 360.0f * FEEDER_GEAR * 8192;
              systime_t error_start_time = chVTGetSystemTime();
              while (chVTIsSystemTimeWithin(error_start_time, (error_start_time + MS2ST(200))) )
              {
                feeder_output = feeder_controlPos(error_angle_sp, FEEDER_OUTPUT_MAX_BACK);
                feeder_canUpdate();
                chThdSleepMilliseconds(1);
              }
            }

            break;
        #endif //FEEDER_USE_BOOST
        default:
            feeder_output = 0;
            feeder_canUpdate();
            break;
    }
}

static THD_WORKING_AREA(feeder_control_wa, 512);
static THD_FUNCTION(feeder_control, p){
    (void) p;
    chRegSetThreadName("feeder controller");
    uint16_t feeder_error_counter;
    while(!chThdShouldTerminateX())
    {
        if(state_count(!(feeder_encode->updated), 100, &feeder_error_counter))
        {
          feeder_error_flag = true;
          system_setErrorFlag();
          chThdExit(MSG_OK);
        }

        if(barrel_info->heatLimit == DRONE_HEATLIMIT){
          level = 3;
          FEEDER_SPEED_SP_RPM = 10  * FEEDER_GEAR * 60 / FEEDER_BULLET_PER_TURN;
        }
        else
        {
          #ifdef RM_DEBUG
            level = -1;
            barrel_info->heatLimit = -1;
            FEEDER_SPEED_SP_RPM = FEEDER_TEST_SETSPEED;
          #else
            //system_setWarningFlag(); //No judgement system data
          #endif
        }

        if(feeder_mode == FEEDER_OVERHEAT){
          if(barrel_info->currentHeatValue < barrel_info->heatLimit - 40){
            feeder_mode = FEEDER_STOP;
          }
        }
        else if(level != -1 && feeder_mode != FEEDER_OVERHEAT &&
          barrel_info->currentHeatValue > barrel_info->heatLimit - 15){
          feeder_brake();
          feeder_mode = FEEDER_OVERHEAT;
        }

        if(
            feeder_mode == FEEDER_STOP &&
            (p_dbus->rc.s1 == RC_S_DOWN || p_dbus->mouse.LEFT)
          )
        {
          feeder_start_time = chVTGetSystemTimeX();

          #ifdef FEEDER_USE_BOOST
            feeder_mode = FEEDER_BOOST;
          #else
            feeder_mode = feeder_fire_mode;// TODO: select fire mode using keyboard input
          #endif
        }
        else if(p_dbus->rc.s1 != RC_S_DOWN && !p_dbus->mouse.LEFT)
        {
          if(feeder_mode == FEEDER_AUTO)
          {
            feeder_brake();
            feeder_mode = FEEDER_STOP;
          }
          else if(feeder_mode == FEEDER_FINISHED)
            feeder_mode = FEEDER_STOP;
        }

        feeder_func();
        chThdSleepMilliseconds(1);
    }
}

static const FEEDER_VEL  = "FEEDER_VEL";
static const FEEDER_POS  = "FEEDER_POS";
static const FEEDER_rest_name = "FEEDER_REST";
static const char subname_feeder_PID[] = "KP KI KD Imax";
void feeder_init(void){

    feeder_encode = can_getFeederMotor();
    barrel_info = can_get_sent_barrelStatus();
    p_dbus = RC_get();

    params_set(&vel_pid, 14,4,FEEDER_VEL,subname_feeder_PID,PARAM_PUBLIC);
    params_set(&pos_pid, 15,4,FEEDER_POS,subname_feeder_PID,PARAM_PUBLIC);
    params_set(&rest_pid, 16,4,FEEDER_rest_name,subname_feeder_PID,PARAM_PUBLIC);

    lpfilter_init(&lp_spd_feeder, 1000, 30);
    feeder_brakePos = (float)feeder_encode->total_ecd;
}

void feeder_start(void)
{
  chThdCreateStatic(feeder_control_wa, sizeof(feeder_control_wa),
                   NORMALPRIO - 5, feeder_control, NULL);
}

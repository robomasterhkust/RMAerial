/**
 * Edward ZHANG, 20171101
 * @file    gimbal.c
 * @brief   Gimbal controller, driver and interface
 */
#include "ch.h"
#include "hal.h"

#include "gimbal.h"
#include "attitude.h"
#include "math_misc.h"

#include "dbus.h"

#ifdef GIMBAL_USE_MAVLINK_CMD
  #include "mavlink_comm.h"
  mavlink_attitude_t* mavlink_attitude;
#endif

#define GIMBAL_IQ_MAX 8000

static pi_controller_t _yaw_vel;
static pi_controller_t _pitch_vel;
static pid_controller_t _yaw_atti;
static pid_controller_t _pitch_atti;
static pid_controller_t _yaw_pos;
static pid_controller_t _pitch_pos;

#define gimbal_canUpdate()   \
  (can_motorSetCurrent(GIMBAL_CAN, GIMBAL_CAN_EID, \
    gimbal.yaw_iq_output, gimbal.pitch_iq_output, 0, 0))

static lpfilterStruct lp_angle[2];
static GimbalStruct gimbal;
static RC_Ctl_t* rc;

static thread_reference_t gimbal_thread_handler = NULL;
static thread_reference_t gimbal_init_thread_handler = NULL;

GimbalStruct* gimbal_get(void)
{
  return &gimbal;
}

uint32_t gimbal_get_error(void)
{
  return gimbal.errorFlag;
}

void gimbal_kill(void)
{
  gimbal.yaw_iq_output = 0;
  gimbal.pitch_iq_output = 0;
  gimbal_canUpdate();
  gimbal.state = GIMBAL_STATE_UNINIT;
}

#define GIMBAL_CV_CMD_TIMEOUT 0.05f
static void gimbal_attiCmd(const float dt, const float yaw_theta1)
{
  static uint16_t cv_wait_count = 0;
  float           rc_input_z = 0.0f, rc_input_y = 0.0f;     //RC input
  static float    cv_input_z = 0.0f, cv_input_y = 0.0f;     //CV input

  const float max_input_z = 2.0f, max_input_y = 2.0f;

  #ifdef GIMBAL_USE_MAVLINK_CMD
    if(mavlinkComm_attitude_check())
    {
      cv_wait_count = 0;
      chSysLock();
      cv_input_z = mavlink_attitude->yawspeed;
      cv_input_y = mavlink_attitude->pitchspeed;
      chSysUnlock();
    }
    else
      cv_wait_count++;

    if(cv_wait_count > (uint16_t)(GIMBAL_CV_CMD_TIMEOUT/dt))
    {
      cv_input_z = 0.0f;
      cv_input_y = 0.0f;
      cv_wait_count = 0;
    }
  #endif

  rc_input_z = -mapInput((float)rc->rc.channel2, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, -max_input_z, max_input_z);
  rc_input_y = -mapInput((float)rc->rc.channel3, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, -max_input_y, max_input_y);

  float input_z = rc_input_z + cv_input_z/cosf(yaw_theta1),
        input_y = rc_input_y + cv_input_y;
  bound(&input_z, max_input_z);
  bound(&input_y, max_input_y);

  /* software limit position*/
  float yaw_speed_limit = gimbal.motor[GIMBAL_YAW]._speed - gimbal.motor[GIMBAL_YAW]._speed_enc,
        pitch_speed_limit = -gimbal.motor[GIMBAL_PITCH]._speed - gimbal.motor[GIMBAL_PITCH]._speed_enc;

  if((gimbal.state & GIMBAL_YAW_AT_UP_LIMIT && input_z > yaw_speed_limit) ||
      (gimbal.state & GIMBAL_YAW_AT_LOW_LIMIT && input_z < yaw_speed_limit))
    input_z = yaw_speed_limit;

  if((gimbal.state & GIMBAL_PITCH_AT_UP_LIMIT && input_y > pitch_speed_limit) ||
      (gimbal.state & GIMBAL_PITCH_AT_LOW_LIMIT && input_y < pitch_speed_limit))
    input_y = pitch_speed_limit;

  input_z *= dt;
  input_y *= -dt;

  float pitch = gimbal.pitch_atti_cmd;
  float yaw = gimbal.yaw_atti_cmd;

  float tanpitch = tanf(pitch);

  /*
   *  Useful simplification : for small x, sin x = tan x = x, cos x = 1
   *  yaw_atti = atan2f(sinf(yaw + input_z) - tanpitch * tantheta * sinf(yaw),
   *                            cosf(yaw + input_z) - tanpitch * tantheta * cosf(yaw));
   *
   *  pitch_atti = asinf(cosf(pitch)*cosf(yaw)*sinf(input_y) + cosf(input_y)*sinf(pitch));
   */
  gimbal.yaw_atti_cmd = atan2f(sinf(yaw + input_z) - tanpitch * input_y * sinf(yaw),
                               cosf(yaw + input_z) - tanpitch * input_y * cosf(yaw));

  gimbal.pitch_atti_cmd = asinf(cosf(pitch)*cosf(yaw)*input_y + sinf(pitch));

  //Avoid gimbal-lock point at pitch = M_PI_2
  bound(&gimbal.pitch_atti_cmd, 1.45f);
}

static void gimbal_checkLimit(void)
{
  if(gimbal.motor[GIMBAL_YAW]._angle < gimbal.axis_limit[0])
    gimbal.state |= GIMBAL_YAW_AT_LOW_LIMIT;
  else if(gimbal.motor[GIMBAL_YAW]._angle > gimbal.axis_limit[1])
    gimbal.state |= GIMBAL_YAW_AT_UP_LIMIT;
  else
    gimbal.state &= ~(GIMBAL_YAW_AT_UP_LIMIT | GIMBAL_YAW_AT_LOW_LIMIT);

  if(gimbal.motor[GIMBAL_PITCH]._angle < gimbal.axis_limit[2])
    gimbal.state |= GIMBAL_PITCH_AT_LOW_LIMIT;
  else if(gimbal.motor[GIMBAL_PITCH]._angle > gimbal.axis_limit[3])
    gimbal.state |= GIMBAL_PITCH_AT_UP_LIMIT;
  else
    gimbal.state &= ~(GIMBAL_PITCH_AT_UP_LIMIT | GIMBAL_PITCH_AT_LOW_LIMIT);
}

#ifdef GIMBAL_ENCODER_USE_SPEED
  #define GIMBAL_SPEED_BUFFER_LEN      50U
  static int16_t _speed_buffer[2][GIMBAL_SPEED_BUFFER_LEN];
  static uint8_t _speed_count_buffer[2][GIMBAL_SPEED_BUFFER_LEN];
  static uint8_t _count_sum[2];
  static uint32_t _speed_count[2];

  static lpfilterStruct lp_axis_ff[2];
#endif

#define GIMBAL_MAX_ANGLE 5.02655f
#define GIMBAL_MIN_ANGLE 1.25664f

#define GIMBAL_ANGLE_PSC 7.6699e-4 //2*M_PI/0x1FFF
#define GIMBAL_CONNECTION_ERROR_COUNT 25U

#define MOTOR_SPEED_ENC_TH_1  0.0f
#define MOTOR_SPEED_ENC_TH_2  0.15f

typedef enum{
  GIMBAL_MOTOR_STOP = 0,
  GIMBAL_MOTOR_CW = 1,
  GIMBAL_MOTOR_CCW = -1
}gimbal_motor_dir_t;

/**
 *  @brief                  process data from gimbal encoder
 *  @param[in,out]  motor   pointer to corresponding motor
 *  @param[in]         id   GIMBAL_YAW or GIMBAL_PITCH
 */
static void gimbal_encoderUpdate(GimbalMotorStruct* motor, uint8_t id)
{
  if(gimbal._encoder[id].updated)
  {

    //Check validiaty of can connection
    gimbal._encoder[id].updated = false;

    float angle_input = gimbal._encoder[id].radian_angle;
    if(id == GIMBAL_YAW)
      angle_input *= GIMBAL_YAW_GEAR;

    motor->_angle = lpfilter_apply(&lp_angle[id], angle_input);
    motor->_current = gimbal._encoder[id].raw_current;

    //add FIR filter to encoder speed
    #ifdef GIMBAL_ENCODER_USE_SPEED
      _count_sum[id] += motor->_wait_count;
      _count_sum[id] -= _speed_count_buffer[id][_speed_count[id] % GIMBAL_SPEED_BUFFER_LEN];

      int16_t diff = gimbal._encoder[id].raw_angle -
        _speed_buffer[id][_speed_count[id] % GIMBAL_SPEED_BUFFER_LEN];

      //Detect zero-crossing scenerio
      if(diff > 6000)
        diff -= 8192;
      else if(diff < -6000)
        diff += 8192;

      motor->_speed_enc = diff * GIMBAL_ANGLE_PSC * GIMBAL_CONTROL_FREQ /_count_sum[id];
      _speed_buffer[id][_speed_count[id] % GIMBAL_SPEED_BUFFER_LEN] = gimbal._encoder[id].raw_angle;
      _speed_count_buffer[id][_speed_count[id] % GIMBAL_SPEED_BUFFER_LEN] = motor->_wait_count;
      _speed_count[id]++;

      /*Dual stabability detector for motor rotating direction estimation
       *-------ccw------------     -------------cw---------
       *              --------stop---------
       */
      if(motor->_dir == GIMBAL_MOTOR_STOP)
      {
        if(motor->_speed_enc > MOTOR_SPEED_ENC_TH_2)
          motor->_dir = GIMBAL_MOTOR_CCW;
        else if(motor->_speed_enc < -MOTOR_SPEED_ENC_TH_2)
          motor->_dir = GIMBAL_MOTOR_CW;
      }
      else if((motor->_dir == GIMBAL_MOTOR_CW && motor->_speed_enc > MOTOR_SPEED_ENC_TH_1) ||
          (motor->_dir == GIMBAL_MOTOR_CCW && motor->_speed_enc < -MOTOR_SPEED_ENC_TH_1))
        motor->_dir = GIMBAL_MOTOR_STOP;

      if(id == GIMBAL_YAW)
        motor->_speed_enc *= GIMBAL_YAW_GEAR;
    #endif

    motor->_wait_count = 1;
  }
  else
  {
    motor->_wait_count++;
    if(motor->_wait_count > GIMBAL_CONNECTION_ERROR_COUNT)
    {
      gimbal.errorFlag |= (GIMBAL_YAW_NOT_CONNECTED << (id == GIMBAL_YAW ? 0 : 1));
      motor->_wait_count = 1;
    }
  }
}

static inline void gimbal_Follow(void)
{
  gimbal.yaw_atti_cmd = gimbal._pIMU->euler_angle[Yaw];
  gimbal.pitch_atti_cmd = gimbal._pIMU->euler_angle[Pitch];
}

static inline float gimbal_controlSpeed (pi_controller_t *const vel,
  volatile GimbalMotorStruct* const motor)
{
  float error = motor->_speed_cmd - motor->_speed;
  vel->error_int += error * vel->ki;
  bound(&(vel->error_int), vel->error_int_max);

  float output = vel->kp * error + vel->error_int;
  return output;
}

#ifdef GIMBAL_ENCODER_USE_SPEED
static inline void gimbal_addFF_int(void)
{
  float yaw_ff = 0.0f, pitch_ff = 0.0f;
  if(gimbal.motor[GIMBAL_YAW]._dir != GIMBAL_MOTOR_STOP)
    yaw_ff = gimbal.axis_ff_int[GIMBAL_YAW] * gimbal.motor[GIMBAL_YAW]._dir;
  else if(gimbal.yaw_iq_cmd > 200.0f)
    yaw_ff = gimbal.axis_ff_int[GIMBAL_YAW];
  else if(gimbal.yaw_iq_cmd < -200.0f)
    yaw_ff = -gimbal.axis_ff_int[GIMBAL_YAW];

  yaw_ff = lpfilter_apply(&lp_axis_ff[GIMBAL_YAW], yaw_ff);

  if(gimbal.motor[GIMBAL_PITCH]._dir != GIMBAL_MOTOR_STOP)
    pitch_ff = gimbal.axis_ff_int[GIMBAL_PITCH] * gimbal.motor[GIMBAL_PITCH]._dir;
  else if(gimbal.pitch_iq_cmd > 200.0f)
    pitch_ff = gimbal.axis_ff_int[GIMBAL_PITCH];
  else if(gimbal.pitch_iq_cmd < -200.0f)
    pitch_ff = -gimbal.axis_ff_int[GIMBAL_PITCH];

  pitch_ff = lpfilter_apply(&lp_axis_ff[GIMBAL_PITCH], pitch_ff);

  gimbal.yaw_iq_cmd += yaw_ff;
  gimbal.pitch_iq_cmd += pitch_ff;
}
#endif

static inline float gimbal_controlAttitude (pid_controller_t *const atti,
                                            const float target_atti,
                                            const float curr_atti,
                                            const float curr_atti_d)
{
  float error = target_atti - curr_atti;
  atti->error_int += error * atti->ki;
  bound(&(atti->error_int), atti->error_int_max);

  //PI-D attitude controller
  float output = atti->kp * error + atti->error_int - atti->kd * curr_atti_d;
  return output;
}

#define GIMBAL_CONTROL_PERIOD_ST     US2ST(1000000U/GIMBAL_CONTROL_FREQ)
static THD_WORKING_AREA(gimbal_thread_wa, 2048);
static THD_FUNCTION(gimbal_thread, p)
{
  (void)p;
  chRegSetThreadName("Gimbal controller");

  chSysLock();
  chThdSuspendS(&gimbal_thread_handler);
  chSysUnlock();

  _yaw_vel.error_int_max = 2000.0f;
  _pitch_vel.error_int_max = 2500.0f;
  _yaw_atti.error_int_max = 4.0f;
  _pitch_atti.error_int_max = 4.0f;

  float pitch_atti_out,yaw_atti_out;
  float sinroll, cosroll, cospitch;

  systime_t tick = chVTGetSystemTimeX();

  while(!chThdShouldTerminateX())
  {
    tick += GIMBAL_CONTROL_PERIOD_ST;
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
      gimbal.errorFlag |= GIMBAL_CONTROL_LOSE_FRAME;
    }

    gimbal_encoderUpdate(&gimbal.motor[GIMBAL_YAW], GIMBAL_YAW);
    gimbal_encoderUpdate(&gimbal.motor[GIMBAL_PITCH], GIMBAL_PITCH);

    gimbal.d_yaw = gimbal.motor[GIMBAL_YAW]._angle - gimbal.axis_init_pos[GIMBAL_YAW];

    /* Variables:
     * yaw_theta1: angle between yaw encoder current value and yaw encoder value at init position
     * yaw_theta2: angle between yaw encoder current value and yaw encoder value at max moment of inertia point
     */
    float yaw_theta1 = gimbal.motor[GIMBAL_PITCH]._angle - gimbal.axis_init_pos[GIMBAL_PITCH];

    gimbal.motor[GIMBAL_PITCH]._speed = gimbal._pIMU->gyroData[Y];
    gimbal.motor[GIMBAL_YAW]._speed = gimbal._pIMU->gyroData[Z] * cosf(yaw_theta1) +
      gimbal._pIMU->gyroData[X] * sinf(yaw_theta1);                 //             ^
                                                                    //             |
    /* TODO Check the sign here----------------------------------------------------- */

    gimbal_checkLimit();

    gimbal_attiCmd(1.0f/GIMBAL_CONTROL_FREQ, yaw_theta1);

    yaw_atti_out = gimbal_controlAttitude(&_yaw_atti,
                                      gimbal.yaw_atti_cmd,
                                      gimbal._pIMU->euler_angle[Yaw],
                                      gimbal._pIMU->d_euler_angle[Yaw]);

    pitch_atti_out = gimbal_controlAttitude(&_pitch_atti,
                                      gimbal.pitch_atti_cmd,
                                      gimbal._pIMU->euler_angle[Pitch],
                                      gimbal._pIMU->d_euler_angle[Pitch]);
    /*
     *  Jacobian output mixer
     *  wy = cos(roll)*pitch' + cos(pitch)*sin(roll)*yaw'
     *  wz = -sin(roll)*pitch' + cos(roll)*cos(pitch)*yaw'
     *  wyaw = wz / cos(theta)
     */

    sinroll = sinf(gimbal._pIMU->euler_angle[Roll]);
    cosroll = cosf(gimbal._pIMU->euler_angle[Roll]);
    cospitch = cosf(gimbal._pIMU->euler_angle[Pitch]);

    //TODO Yaw mixer is not aligned

    gimbal.motor[GIMBAL_YAW]._speed_cmd = -sinroll * pitch_atti_out + cospitch * cosroll * yaw_atti_out;

    if(cosf(yaw_theta1) > 0.1f)
      gimbal.motor[GIMBAL_YAW]._speed_cmd /= cosf(yaw_theta1);
    else
      gimbal.motor[GIMBAL_YAW]._speed_cmd = 0.0f;

    gimbal.motor[GIMBAL_PITCH]._speed_cmd = cosroll * pitch_atti_out + cospitch * sinroll * yaw_atti_out;

    gimbal.yaw_iq_cmd = gimbal_controlSpeed(&_yaw_vel, &gimbal.motor[GIMBAL_YAW]);
    gimbal.pitch_iq_cmd = gimbal_controlSpeed(&_pitch_vel, &gimbal.motor[GIMBAL_PITCH]);

    /*
     *  @brief      Utilize the gimbal kinamatics model and add a feed-forward output
     *  @NOTE       Requires accelerometer raw data
     */
    float ff_pitch_ext = norm_vector3_projection(gimbal._pIMU->accelData, gimbal.axis_ff_accel);
    gimbal.pitch_iq_cmd += gimbal.axis_ff_ext[GIMBAL_PITCH] * ff_pitch_ext;

    /*
     * the reference acceleration vector is calculated in this way
     * RefX = G * cos(tan-1(M2*cos(theta)/M1)) * cos(theta + pitch0)
     * RefY = G * sin(tan-1(M2*cos(theta)/M1))
     * RefZ = G * cos(tan-1(M2*cos(theta)/M1)) * sin(theta + pitch0)
     * useful simplification : cos(tan-1(x)) = 1/sqrt(1+x^2)
     *                         sin(tan-1(x)) = x/sqrt(1+x^2)
     */

/*  float yaw_theta2 = gimbal.axis_ff_weight[4]*cosf(yaw_theta1) / gimbal.axis_ff_weight[GIMBAL_YAW];
    float yaw_theta3 = yaw_theta1 + gimbal.axis_ff_weight[5];

    float temp = sqrtf(1 + yaw_theta2 * yaw_theta2);

    float a_ref[3];
    a_ref[X] = -(GRAV / temp) * cosf(yaw_theta3);
    a_ref[Y] = -(GRAV * yaw_theta2) / temp;
    a_ref[Z] = -(GRAV / temp) * sinf(yaw_theta3);
    float ff_yaw_ext = norm_vector3_projection(gimbal._pIMU->accelData, a_ref);
    gimbal.yaw_iq_cmd +=
      (gimbal.axis_ff_weight[GIMBAL_YAW] + gimbal.axis_ff_weight[4] * cosf(yaw_theta1)) * ff_yaw_ext;



    //gimbal_addFF_int();
*/

    /* currently we cannot determine the max moment of inertia point*/
    float yaw_theta2 = yaw_theta1;
    //Scale down gimbal yaw power according to pitch angle
    //Moment of inertia decreases as theta1 moves away from 0
    gimbal.yaw_iq_cmd -=
      (sinf(yaw_theta2) * sinf(yaw_theta2) * gimbal.axis_ff_ext[2]) * gimbal.yaw_iq_cmd;

    /*output limit*/
    gimbal.yaw_iq_output = boundOutput(gimbal.yaw_iq_cmd, GIMBAL_IQ_MAX);
    gimbal.pitch_iq_output = boundOutput(gimbal.pitch_iq_cmd, GIMBAL_IQ_MAX);

    gimbal_canUpdate();

    //Stop the thread while calibrating IMU
    if(gimbal._pIMU->accelerometer_not_calibrated || gimbal._pIMU->gyroscope_not_calibrated)
    {
      gimbal_kill();
      chThdExit(MSG_OK);
    }
  }
}

static inline float gimbal_controlPos(pid_controller_t* const controller,
  const float _error, const float _d_error)
{
  controller->error_int += _error * controller->ki;
  bound(&(controller->error_int), controller->error_int_max);

  float output = _error * controller->kp + controller->error_int - _d_error * controller->kd;
  bound(&output, 3500);

  return output;
}

#define GIMBAL_INIT_MAX_ERROR         2e-2
#define GIMBAL_INIT_SCORE_FULL        100U
static THD_WORKING_AREA(gimbal_init_thread_wa, 2048);
static THD_FUNCTION(gimbal_init_thread, p)
{
  (void)p;
  chRegSetThreadName("Gimbal init thread");

  float _error[2];
  uint8_t i;

  _pitch_pos.error_int_max = 1500.0f;
  _yaw_pos.error_int_max = 1000.0f;

  uint16_t _init_score[2] = {0, 0};

  systime_t tick = chVTGetSystemTimeX() + MS2ST(500);
  while(true)
  {
    gimbal_encoderUpdate(&gimbal.motor[GIMBAL_YAW], GIMBAL_YAW);
    gimbal_encoderUpdate(&gimbal.motor[GIMBAL_PITCH], GIMBAL_PITCH);

    _error[GIMBAL_YAW] = gimbal.axis_init_pos[GIMBAL_YAW] - gimbal.motor[GIMBAL_YAW]._angle;
    _error[GIMBAL_PITCH] = gimbal.axis_init_pos[GIMBAL_PITCH] - gimbal.motor[GIMBAL_PITCH]._angle;

    while(_error[GIMBAL_YAW] > M_PI * GIMBAL_YAW_GEAR)
      _error[GIMBAL_YAW] -= 2 * M_PI * GIMBAL_YAW_GEAR;
    while(_error[GIMBAL_YAW] < -M_PI * GIMBAL_YAW_GEAR)
      _error[GIMBAL_YAW] += 2 * M_PI * GIMBAL_YAW_GEAR;

    while(_error[GIMBAL_PITCH] > M_PI)
      _error[GIMBAL_PITCH] -= 2 * M_PI;
    while(_error[GIMBAL_PITCH] < -M_PI)
      _error[GIMBAL_PITCH] += 2 * M_PI;

    if(chVTGetSystemTimeX() > tick)
      gimbal.yaw_iq_cmd = gimbal_controlPos(&_yaw_pos, _error[GIMBAL_YAW],
        gimbal.motor[GIMBAL_YAW]._speed_enc);

    gimbal.pitch_iq_cmd = gimbal_controlPos(&_pitch_pos, _error[GIMBAL_PITCH],
      gimbal.motor[GIMBAL_PITCH]._speed_enc);

    gimbal.yaw_iq_output = gimbal.yaw_iq_cmd;
    gimbal.pitch_iq_output = gimbal.pitch_iq_cmd;

    gimbal_canUpdate();

    //evaluation of init status
    for (i = 0; i < 2; i++)
    {
      if(_error[i] < GIMBAL_INIT_MAX_ERROR && _error[i] > -GIMBAL_INIT_MAX_ERROR)
        _init_score[i]++;
      else if(_init_score[i] > 100U)
        _init_score[i] -= 50;
      else
        _init_score[i] = 0;
    }

    #ifndef GIMBAL_INIT_TEST
      if(_init_score[0] > GIMBAL_INIT_SCORE_FULL &&
         _init_score[1] > GIMBAL_INIT_SCORE_FULL &&
          gimbal._pIMU->state == IMU_STATE_READY)
      {
          /*exit this thread and start attitude control*/
        chSysLock();

        gimbal_Follow();

        chThdResumeS(&gimbal_thread_handler, MSG_OK);
        chThdExitS(MSG_OK);
        chSysUnlock();
      }
    #endif

    chThdSleepMilliseconds(1);
  }
}

/* name of gimbal parameters*/
const char axis_ff_name[]  = "Gimbal Axis FF";
const char init_pos_name[] = "Gimbal Init Pos";
const char accl_name[]     = "Gimbal FF Accl";
const char _yaw_vel_name[] =   "Gimbal Yaw Vel";
const char _pitch_vel_name[] = "Gimbal Pitch Vel";
const char _yaw_atti_name[] =   "Gimbal Yaw Atti";
const char _pitch_atti_name[] = "Gimbal Pitch Atti";
const char yaw_pos_name[] =   "Gimbal Yaw Pos";
const char pitch_pos_name[] = "Gimbal Pitch Pos";
const char ff_int_name[] = "Gimbal FF Int";
const char limit_name[] = "Gimbal axis limit";

const char subname_axis[]  = "Yaw Pitch";
const char subname_ff[]    = "Yaw_w1 Pitch_w Yaw_SD Pitch_a Yaw_w2 Yaw_th";
const char subname_accl[]  = "YawX YawY YawZ PitchX PitchY PitchZ";
const char limit_subname[] = "Yaw_min Yaw_max Pitch_min Pitch_max";


/*
 *  @brief      Initialize the gimbal motor driver
 *  @NOTE       Requires to run can_processInit() first
 *
 *  @api
 */
void gimbal_init(void)
{
  memset(&gimbal, 0 ,sizeof(GimbalStruct));

  memset(&_yaw_pos,    0 ,sizeof(pid_controller_t));
  memset(&_pitch_pos,  0 ,sizeof(pid_controller_t));
  memset(&_yaw_vel,    0 ,sizeof(pi_controller_t));
  memset(&_pitch_vel,  0 ,sizeof(pi_controller_t));
  memset(&_yaw_atti,   0 ,sizeof(pid_controller_t));
  memset(&_pitch_atti, 0 ,sizeof(pid_controller_t));

  gimbal_kill();

  gimbal._pIMU = imu_get();
  rc = RC_get(RC_INDEX_GIMBAL);
  gimbal._encoder = can_getGimbalMotor();
  chThdSleepMilliseconds(3);

  lpfilter_init(&lp_angle[GIMBAL_YAW], GIMBAL_CONTROL_FREQ, GIMBAL_CUTOFF_FREQ);
  lpfilter_init(&lp_angle[GIMBAL_PITCH], GIMBAL_CONTROL_FREQ, GIMBAL_CUTOFF_FREQ);

  #ifdef GIMBAL_ENCODER_USE_SPEED
    lpfilter_init(&lp_axis_ff[GIMBAL_YAW], GIMBAL_CONTROL_FREQ, 10);
    lpfilter_init(&lp_axis_ff[GIMBAL_PITCH], GIMBAL_CONTROL_FREQ, 10);
  #endif

  gimbal_encoderUpdate(&gimbal.motor[GIMBAL_YAW], GIMBAL_YAW);
  gimbal_encoderUpdate(&gimbal.motor[GIMBAL_PITCH], GIMBAL_PITCH);

  params_set(gimbal.axis_init_pos,  5, 2,   init_pos_name,  subname_axis,    PARAM_PUBLIC);
  params_set(gimbal.axis_ff_ext,    2, 6,   axis_ff_name,   subname_ff,      PARAM_PUBLIC);
  params_set(gimbal.axis_ff_accel,  6, 6,   accl_name,      subname_accl,    PARAM_PUBLIC);
  params_set(gimbal.axis_ff_int,    9, 2,   ff_int_name,    subname_axis,    PARAM_PUBLIC);
  params_set(gimbal.axis_limit,     10, 4,  limit_name,     limit_subname,   PARAM_PUBLIC);

  params_set(&_yaw_pos,   3, 3, yaw_pos_name,   subname_PID,   PARAM_PUBLIC);
  params_set(&_pitch_pos, 4, 3, pitch_pos_name, subname_PID,   PARAM_PUBLIC);

  params_set(&_yaw_vel,     0, 2, _yaw_vel_name,   subname_PI,      PARAM_PUBLIC);
  params_set(&_pitch_vel,   1, 2, _pitch_vel_name, subname_PI,      PARAM_PUBLIC);

  params_set(&_yaw_atti,     7, 3, _yaw_atti_name,   subname_PID,      PARAM_PUBLIC);
  params_set(&_pitch_atti,   8, 3, _pitch_atti_name, subname_PID,      PARAM_PUBLIC);

  #ifdef GIMBAL_USE_MAVLINK_CMD
    mavlink_attitude = mavlinkComm_attitude_subscribe();
  #endif

  chThdCreateStatic(gimbal_init_thread_wa, sizeof(gimbal_init_thread_wa),
                    NORMALPRIO - 5, gimbal_init_thread, NULL);
  chThdCreateStatic(gimbal_thread_wa, sizeof(gimbal_thread_wa),
                    NORMALPRIO - 5, gimbal_thread, NULL);

  gimbal_canUpdate();
  gimbal.state = GIMBAL_STATE_INITING;
}

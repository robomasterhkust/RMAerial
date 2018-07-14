/**
 * Edward ZHANG
 * @file    shellcfg.c
 * @brief   definitions of shell command functions
 */
#include "ch.h"

#include "main.h"
#include "shell.h"
#include <string.h>

#define SERIAL_CMD       &SDU1
#define SERIAL_DATA      &SDU1

static thread_t* matlab_thread_handler = NULL;
/**
 * @brief Transmit uint32_t and float through serial port to host machine
 * @require Initialization of ChibiOS serial driver before using this function
 *
 * @param[in] chp         pointer to a @p BaseSequentialStream implementing object
 * @param[in] txbuf_d     array of 32-bit integers to tramsmit, can be signed or unsigned
 * @param[in] txbuf_f     array of float point numbers to tramsmit
 * @param[in] num_int     number of 32-bit integers to tramsmit
 * @param[in] num_float   number of float point numbers to tramsmit
 *
 * @TODO improve the transmission protocol to enable easier setup for the host machine
 */
#define SYNC_SEQ  0xaabbccdd
static void transmit_matlab
  (BaseSequentialStream* chp,
    uint32_t* const txbuf_d, float* const txbuf_f,
    const uint8_t num_int, const uint8_t num_float)
{
  uint32_t sync = SYNC_SEQ;
  char* byte = (char*)&sync;

  uint8_t i;
  for (i = 0; i < 4; i++)
    chSequentialStreamPut(chp, *byte++);

  byte = (char*)txbuf_d;
  for (i = 0; i < 4*num_int; i++)
    chSequentialStreamPut(chp, *byte++);

  byte = (char*)txbuf_f;
  for (i = 0; i < 4*num_float; i++)
    chSequentialStreamPut(chp, *byte++);
}

#define HOST_TRANSMIT_FREQ  100U
static THD_WORKING_AREA(matlab_thread_wa, 512);
static THD_FUNCTION(matlab_thread, p)
{
  (void)p;
  chRegSetThreadName("matlab tramsmitter");

  int32_t txbuf_d[16];
  float txbuf_f[16];
  BaseSequentialStream* chp = (BaseSequentialStream*)SERIAL_DATA;

  PIMUStruct pIMU = adis16470_get();
  GimbalStruct* gimbal = gimbal_get();

  osdk_velocity* vel = osdk_velocity_subscribe();

  uint32_t tick = chVTGetSystemTimeX();
  const uint16_t period = US2ST(1000000/HOST_TRANSMIT_FREQ);
  while (!chThdShouldTerminateX())
  {
    tick += period;
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    txbuf_f[0] = gimbal->motor[GIMBAL_YAW]._speed;

    transmit_matlab(chp, NULL, txbuf_f, 0, 1);
  }
}

/*===========================================================================*/
/* Definitions of shell command functions                                    */
/*===========================================================================*/
static THD_WORKING_AREA(Shell_thread_wa, 1024);
void cmd_test(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;
  PIMUStruct pIMU = adis16470_get();
  GimbalStruct* gimbal = gimbal_get();

  chprintf(chp, "Yaw: %f\r\n",  gimbal->motor[GIMBAL_YAW]._speed);
  chprintf(chp, "YawEnc: %f\r\n", gimbal->motor[GIMBAL_YAW]._speed_enc);
  chprintf(chp, "Pitch: %f\r\n", gimbal->motor[GIMBAL_PITCH]._speed);
  chprintf(chp, "PitchEnc: %f\r\n\n\n\n", gimbal->motor[GIMBAL_PITCH]._speed_enc);
}

void cmd_judgeTest(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;

  judge_fb_t* judge = judgeDataGet();
  chprintf(chp, "Drone HP: %d\r\n", judge->gameInfo.remainHealth);
  chprintf(chp, "Full HP: %d\r\n", judge->gameInfo.fullHealth);
  chprintf(chp, "Game status: %d\r\n", judge->gameInfo.gameStatus);
  chprintf(chp, "Robot Level: %d\r\n", judge->gameInfo.robotLevel);
  chprintf(chp, "Heat: %d\r\n", judge->powerInfo.shooterHeat0);
}


void cmd_error(BaseSequentialStream * chp, int argc, char *argv[])
{
  uint32_t error;

  //Initialization error
  //error = init_state_get();
  //if(error & INIT_SEQUENCE_3_RETURN_1)
  //  chprintf(chp,"E:INIT SEQ 3 FAILED -- GIMBAL YAW NOT CONNECTED\r\n");
  //if(error & INIT_SEQUENCE_3_RETURN_2)
  //  chprintf(chp,"E:INIT SEQ 3 FAILED -- GIMBAL PITCH NOT CONNECTED\r\n");

  //Gimbal error
  error = gimbal_get_error();
  if(error & GIMBAL_YAW_NOT_CONNECTED)
    chprintf(chp,"E:GIMBAL YAW CONNECTION LOST\r\n");
  if(error & GIMBAL_PITCH_NOT_CONNECTED)
    chprintf(chp,"E:GIMBAL PITCH CONNECTION LOST\r\n");
  if(error & GIMBAL_CONTROL_LOSE_FRAME)
    chprintf(chp,"W:Gimbal control lose frame\r\n");

  //IMU error
  error = adis16470_get_error();
  PIMUStruct pIMU = adis16470_get();
  if(error & ADIS16470_AXIS_CONF_ERROR)
    chprintf(chp,"E:IMU COORDINATE CONFIGURATION ERROR\r\n");
  if(error & ADIS16470_SENSOR_ERROR)
    chprintf(chp,"E:ADIS16470 SENSOR ERROR: %X\r\n", pIMU->diag_stat);
  if(error & ADIS16470_READING_ERROR)
    chprintf(chp,"E:ADIS16470 READING ERROR\r\n");
  if(error & ADIS16470_UNCONNECTED)
    chprintf(chp,"E:ADIS16470 SENSOR UNCONNECTED\r\n");
  if(error & ADIS16470_ACCEL_NOT_CALIBRATED)
    chprintf(chp,"W:ADIS16470 Accelerometer not calibrated\r\n");
  if(error & ADIS16470_GYRO_NOT_CALIBRATED)
    chprintf(chp,"W:ADIS16470 Gyroscope not calibrated\r\n");
  if(error & ADIS16470_DATA_INVALID)
    chprintf(chp,"W:ADIS16470 has invalid reading\r\n");
  if(error & ADIS16470_LOSE_FRAME)
    chprintf(chp,"W:Attitude estimator lose frame\r\n");
  adis16470_clear_error();

  //feeder error
  if(feeder_get_error())
    chprintf(chp, "E: FEEDER MOTOR NOT CONNECTED\r\n");

  if(osdkComm_getError() & OSDK_RX_TIMEOUT)
    chprintf(chp, "W: OSDK RX TIMEOUT OCCURED\r\n");

  system_clearWarningFlag();
}

void cmd_getFWVersion(BaseSequentialStream * chp, int argc, char *argv[])
{
  uint8_t cmd_val = 0xFF;
  osdk_StartTX_ACK(&cmd_val, 1,
    OSDK_ACTIVATION_SET, OSDK_FWVERSION_ID, OSDK_TX_WAIT);
}

void cmd_activate(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;
  chprintf(chp,"Activate: %x\r\n",osdk_activate(OSDK_APP_ID));
}

#ifdef OSDK_AUTOMATION_TEST_SAFE
  void cmd_grabCtrl(BaseSequentialStream * chp, int argc, char *argv[])
  {
    droneCmd_OSDK_control(ENABLE);
  }

  void cmd_ARM(BaseSequentialStream * chp, int argc, char *argv[])
  {
    droneCmd_armMotor_control(ENABLE);
  }

  void cmd_flight_ctrl(BaseSequentialStream * chp, int argc, char *argv[])
  {
    const uint8_t ctrl_mode = OSDK_FLIGHT_MODE_HORI_VEL   |
                              OSDK_FLIGHT_MODE_VERT_VEL   |
                              OSDK_FLIGHT_MODE_YAW_RATE   |
                              OSDK_FLIGHT_MODE_BODY_FRAME |
                              OSDK_FLIGHT_MODE_STABLE;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float yaw = 0.0f;

    if(argc)
    {
      if(!strcmp(argv[0], "LEFT"))
        y = -2.0f;
      else if(!strcmp(argv[0], "RIGHT"))
        y = 2.0f;
      else if(!strcmp(argv[0], "FRONT"))
        x = 2.0f;
      else if(!strcmp(argv[0], "BACK"))
        x = -2.0f;
      else if(!strcmp(argv[0], "UP"))
        z = 4.0f;
      else if(!strcmp(argv[0], "DOWN"))
        z = -4.0f;
      else if(!strcmp(argv[0], "CW"))
        yaw = 50.0f;
      else if(!strcmp(argv[0], "CCW"))
        yaw = -50.0f;

      chprintf(chp, "Sending flight command: %s\r\n", argv[0]);
      uint16_t i = 0;
      while(i++ < 500)
      {
        droneCmd_Flight_control(0b01001010, x, y, z, yaw);
        chThdSleepMilliseconds(2);
      }
    }
  }
#endif

/**
 * @brief Start the data tramsmission to matlab
 * @note caution of data flooding to the serial port
 */
void cmd_data(BaseSequentialStream * chp, int argc, char *argv[])
{
  uint8_t sec = 10;

  if(argc && matlab_thread_handler == NULL)
  {
    char *toNumber = argv[0];
    uint32_t finalNum=0;
    while(*toNumber>='0' && *toNumber<='9')
      finalNum=finalNum*10+*(toNumber++)-'0';

    if(finalNum == 0)
      finalNum = 10;

    sec = (finalNum < 60 ? finalNum : 60);

    chprintf(chp,"Data transmission start in %d seconds...\r\n", sec);
    chThdSleepSeconds(sec);

    matlab_thread_handler = chThdCreateStatic(matlab_thread_wa, sizeof(matlab_thread_wa),
        NORMALPRIO - 3,
        matlab_thread, NULL);
  }
  else if(matlab_thread_handler != NULL)
  {
    chThdTerminate(matlab_thread_handler);
    matlab_thread_handler = NULL;
  }
}

/**
 * @brief array of shell commands, put the corresponding command and functions below
 * {"command", callback_function}
 */
static const ShellCommand commands[] =
{
  {"test", cmd_test},
  {"activate", cmd_activate},
  {"fwVersion", cmd_getFWVersion},
  {"judge", cmd_judgeTest},
  {"WTF", cmd_error},
  {"\xEE", cmd_data},
  #ifdef PARAMS_USE_USB
    {"\xFD",cmd_param_scale},
    {"\xFB",cmd_param_update},
    {"\xFA",cmd_param_tx},
    {"\xF9",cmd_param_rx},
  #endif
  #ifdef OSDK_AUTOMATION_TEST_SAFE
    {"grabCtrl", cmd_grabCtrl},
    {"ARM!", cmd_ARM},
    {"FLY",  cmd_flight_ctrl},
  #endif
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 =
{
  (BaseSequentialStream *)SERIAL_CMD,
  commands
};

/**
 * @brief start the shell service
 * @require enable the corresponding serial ports in mcuconf.h and board.h
 *          Select the SERIAL_CMD port in main.h
 *
 * @api
 */
void shellStart(void)
{
  //sdStart(SERIAL_CMD, NULL);
  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */

  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);

  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  shellInit();

  shellCreateStatic(&shell_cfg1, Shell_thread_wa,
      sizeof(Shell_thread_wa), NORMALPRIO);

}

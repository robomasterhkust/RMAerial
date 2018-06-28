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

    txbuf_f[0] = osdk_attitude_get_yaw();

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
  chprintf(chp,"yaw: %f\r\n",osdk_attitude_get_yaw());

}

void cmd_activate(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;
  chprintf(chp,"Activate: %x\r\n",droneCmd_activate(OSDK_APP_ID));
}

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

void cmd_calibrate(BaseSequentialStream * chp, int argc, char *argv[])
{
  PIMUStruct pIMU = adis16470_get();

  int32_t accelBias[3], gyroBias[3];

  if(pIMU->state == ADIS16470_READY)
  {
    pIMU->state = ADIS16470_CALIBRATING;
    chThdSleepMilliseconds(10);

    adis16470_get_gyro_bias(gyroBias);
    adis16470_get_accel_bias(accelBias);
  }
  else
  {
    chprintf(chp, "IMU initialization not complete \\ Error occured\r\n");
    return;
  }

  if(argc)
  {
    gimbal_kill();
    chprintf(chp, "Calibration in process...\r\n");
    chThdSleepSeconds(2);

    if(!strcmp(argv[0], "accl"))
    {
      adis16470_reset_calibration();
      calibrate_accelerometer(pIMU, accelBias);

      chprintf(chp, "Saving to ADIS16470 flash...\r\n");
      //adis16470_set_calibration_id(pIMU->calibration_id, 0);
      adis16470_bias_update(accelBias, gyroBias);
    }
    else if(!strcmp(argv[0], "gyro"))
    {
      adis16470_reset_calibration();

      calibrate_gyroscope(pIMU, gyroBias);

      chprintf(chp, "Saving to ADIS16470 flash...\r\n");
      //adis16470_set_calibration_id(0, pIMU->calibration_id);
      adis16470_bias_update(accelBias, gyroBias);
    }
    else if(!strcmp(argv[0], "res"))
    {
      chprintf(chp, "Restoring factory calibration\r\n");
      adis16470_reset_calibration();
      chprintf(chp, "Saving to ADIS16470 flash...\r\n");
      adis16470_bias_update(accelBias, gyroBias);
    }
  }
  else
    chprintf(chp,"Calibration: gyro, accl\r\n");

  pIMU->state = ADIS16470_READY;
  chThdResume(&(pIMU->imu_Thd), MSG_OK);
  pIMU->imu_Thd = NULL;
}

/**
 * @brief array of shell commands, put the corresponding command and functions below
 * {"command", callback_function}
 */
static const ShellCommand commands[] =
{
  {"test", cmd_test},
  {"activate", cmd_activate},
  {"cal", cmd_calibrate},
  {"\xEE", cmd_data},
  #ifdef PARAMS_USE_USB
    {"\xFD",cmd_param_scale},
    {"\xFB",cmd_param_update},
    {"\xFA",cmd_param_tx},
    {"\xF9",cmd_param_rx},
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

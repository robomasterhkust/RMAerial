#include "ch.h"
#include "hal.h"

#include "osdk_comm.h"
#include "math.h"

static osdk_timeStamp  timestamp;
static bool            timestamp_subscribed = false;
static bool            timestamp_received   = false;

osdk_timeStamp* osdk_timeStamp_subscribe(void)
{
  timestamp_subscribed = true;
  return &timestamp;
}

bool osdk_timeStamp_check(void)
{
  bool result = timestamp_received;
  timestamp_received = false;
  return result;
}

static osdk_quaternion quaternion;
static bool            attitude_subscribed = false;
static bool            attitude_received   = false;

osdk_quaternion* osdk_attitude_subscribe(void)
{
  attitude_subscribed = true;
  return &quaternion;
}

bool osdk_attitude_check(void)
{
  bool result = attitude_received;
  attitude_received = false;
  return result;
}

float osdk_attitude_get_yaw(void)
{
  chSysLock();
  attitude_received = false;
  float yaw = atan2f(2.0f * (quaternion.q0 * quaternion.q3 + quaternion.q1 * quaternion.q2),
               1.0f - 2.0f * (quaternion.q2 * quaternion.q2 + quaternion.q3 * quaternion.q3));
  chSysUnlock();

  return yaw;
}

void _osdk_topic_decode(const osdk_flight_data_t* const flight_data)
{
  uint8_t index = 0;
  if(flight_data->flag.timestamp)
  {
    timestamp_received = true;
    if(timestamp_subscribed)
    {
      chSysLock();
      memcpy(&timestamp, &(flight_data->data[index]), 9);
      chSysUnlock();
    }
    index += 9;
  }
  if(flight_data->flag.attitude)
  {
    attitude_received = true;
    if(attitude_subscribed)
    {
      chSysLock();
      memcpy(&quaternion, &(flight_data->data[index]), 16);
      chSysUnlock();
    }
    index += 16;
  }
}

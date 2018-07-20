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
static float           yaw;
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
  attitude_received = false;
  return yaw;
}

static osdk_velocity   velocity;
static bool            velocity_subscribed = false;
static bool            velocity_received   = false;

osdk_RC* osdk_velocity_subscribe(void)
{
  velocity_subscribed = true;
  return &velocity;
}

bool osdk_velocity_check(void)
{
  bool result = velocity_received;
  velocity_received = false;
  return result;
}

static osdk_RC         rc;
static bool            rc_subscribed = false;
static bool            rc_received   = false;

osdk_RC* osdk_RC_subscribe(void)
{
  rc_subscribed = true;
  return &rc;
}

bool osdk_rc_check(void)
{
  bool result = rc_received;
  rc_received = false;
  return result;
}

static uint8_t         battery_data;
static osdk_battery    battery;
static bool            battery_subscribed = false;
static bool            battery_received   = false;

osdk_battery* osdk_battery_subscribe(void)
{
  battery_subscribed = true;
  return &battery;
}

bool osdk_battery_check(void)
{
  bool result = battery_received;
  battery_received = false;
  return result;
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
      memcpy(&timestamp, &(flight_data->data[index]), sizeof(osdk_timeStamp));
      chSysUnlock();
    }
    index += sizeof(osdk_timeStamp);
  }
  if(flight_data->flag.attitude)
  {
    attitude_received = true;
    if(attitude_subscribed)
    {
      chSysLock();
      memcpy(&quaternion, &(flight_data->data[index]), sizeof(osdk_quaternion));
      yaw = atan2f(2.0f * (quaternion.q0 * quaternion.q3 + quaternion.q1 * quaternion.q2),
            1.0f - 2.0f * (quaternion.q2 * quaternion.q2 + quaternion.q3 * quaternion.q3));
      chSysUnlock();
    }
    index += sizeof(osdk_quaternion);
  }
  if(flight_data->flag.linear_velocity)
  {
    if(velocity_subscribed)
    {
      chSysLock();
      memcpy(&velocity, &(flight_data->data[index]), sizeof(osdk_velocity));
      chSysUnlock();
    }
    index += sizeof(osdk_velocity);
  }
  if(flight_data->flag.position)
  {
    index += sizeof(osdk_globalPosition);
  }
  if(flight_data->flag.RC)
  {
    rc_received = true;
    if(rc_subscribed)
    {
      chSysLock();
      memcpy(&rc, &(flight_data->data[index]), sizeof(osdk_RC));
      chSysUnlock();
    }
    index += sizeof(osdk_RC);
  }
  if(flight_data->flag.flight_status)
  {
    index += 1;
  }
  if(flight_data->flag.battery)
  {
    battery_received = true;
    if(battery_subscribed)
    {
      chSysLock();
      memcpy(&battery_data, &(flight_data->data[index]), 1);
      chSysUnlock();
    }
    index += 1;
  }
}

#ifndef _OSDK_PROTOCOL_SIM_H_
#define _OSDK_PROTOCOL_SIM_H_

/* Currently only support length no greater than 255*/
#define OSDK_MAX_PACKET_LEN 100U

#define OSDK_STX 0xAA
#define OSDK_NO_PAYLOAD_LEN  16U

#define OSDK_PUSH_DATA_SET   0x02
#define OSDK_FLIGHT_DATA_ID  0x00

typedef struct
{
  bool timestamp          : 1;
  bool attitude           : 1;
  bool accleration        : 1;
  bool linear_velocity    : 1;
  bool angular_velocity   : 1;
  bool position           : 1;
  bool magnatometer       : 1;
  bool RC                 : 1;
  bool gimbal             : 1;
  bool flight_status      : 1;
  bool battery            : 1;
  bool device             : 1;
  uint8_t shit            : 4; //Useless
} __attribute__((packed)) osdk_flight_data_flag_t;

typedef struct
{
  uint8_t                   set;
  uint8_t                    id;
  osdk_flight_data_flag_t  flag;
  uint8_t                  data[OSDK_MAX_PACKET_LEN - OSDK_NO_PAYLOAD_LEN - 4];
} __attribute__((packed)) osdk_flight_data_t;

typedef struct
{
  uint8_t   start;
  uint16_t  len     : 10 ;
  uint8_t   shit0   : 6  ;//Useless
  uint8_t   session : 5  ;
  uint8_t   ack     : 1  ;
  uint8_t   shit1   : 2  ;//Useless
  uint8_t   shit2[4];     //Useless
  uint16_t  seq;
  uint16_t  crc16;
  uint8_t   data[OSDK_MAX_PACKET_LEN - OSDK_NO_PAYLOAD_LEN];
  //uint32_t  crc32;
} __attribute__((packed)) osdk_frame_t;

/*! @brief struct for TOPIC_QUATERNION
 *
 */
typedef struct
{
  float q0; /*!< w */
  float q1; /*!< x */
  float q2; /*!< y */
  float q3; /*!< z */
} osdk_quaternion;   // pack(1)

/*! @brief struct for TOPIC_QUATERNION
 *
 */
typedef struct
{
  float roll; /*!< w */
  float pitch; /*!< x */
  float yaw; /*!< y */
} osdk_attitude_eulerZYX;   // pack(1)

/*!
 * @brief struct for multiple Topics
 */
typedef struct
{
  float x;
  float y;
  float z;
} osdk_vector3f; // pack(1)

/*!
 * @brief struct for multiple Topics
 *
 * @note for TOPIC_GPS_POSITION, data type: (uint32)deg*10^7
 */
typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} osdk_vector3d; // pack(1)

/*!
 * @brief struct for data broadcast, timestamp from local cache
 *
 * @note not available in data subscription
 */
typedef struct
{
  uint32_t time_ms;
  uint32_t time_ns;
} osdk_timeStamp; // pack(1)

/*!
 * @brief struct for data broadcast, software sync timestamp from local cache
 *
 * @note not available in data subscription and different from Hardware sync
 */
typedef struct
{
  uint32_t time_2p5ms; /*!< relative sync time */
  uint16_t tag;
  uint8_t  flag;
} osdk_syncStamp; // pack(1)

/*!
 * @brief struct indicates the signal level of GPS velocity info <br>
 *
 */
typedef struct
{

  uint8_t health : 1; /*!< 1 - using GPS, 0 - not using GPS */
  uint8_t reserve : 7;
} osdk_velocityInfo; // pack(1)

/*!
 * @brief struct for TOPIC_VELOCITY
 *
 * @note The velocity may be in body or ground frame
 * based on settings in DJI Assistant 2's SDK page.
 */
typedef struct
{
  osdk_vector3f data;
  /*! scale from 0 - 5 signifying gps signal strength <br>
   *  greater than 3 for strong signal
   */
  osdk_velocityInfo info;
} osdk_velocity; // pack(1)

/*!
 * @brief struct for data broadcast, return GPS data
 *
 * @note not available in data subscription
 */
typedef struct
{
  uint8_t WTF[16]; //We do not need that shit!
  float altitude;  /*!< Measured by barometer: WGS 84 reference ellipsoid */
  float height;    /*!< Ultrasonic height in meters */
  uint8_t   health;    /*!< scale from 0 - 5 signifying gps signal strength <br>
                        * greater than 3 for strong signal */
} osdk_globalPosition;      // pack(1)

/*!
 * @brief struct for TOPIC_GPS_FUSED
 *
 * @note fusion data from GPS and IMU, return in gps format
 */
typedef struct
{
  uint8_t WTF[16]; //We do not need that shit!
  float altitude;               /*!< WGS 84 reference ellipsoid */
  uint16_t  visibleSatelliteNumber; /*!< number of visible satellite */
} osdk_GPSFused;                         // pack(1)

/*!
 * @brief struct for data broadcast, return obstacle info around the vehicle
 *
 * @note available in M210 (front, up, down)
 */
typedef struct
{
  float down;            /*!< distance from obstacle (cm) */
  float front;           /*!< distance from obstacle (cm) */
  float right;           /*!< distance from obstacle (cm) */
  float back;            /*!< distance from obstacle (cm) */
  float left;            /*!< distance from obstacle (cm) */
  float up;              /*!< distance from obstacle (cm) */
  uint8_t   downHealth : 1;  /*!< Down sensor flag: 0 - not working, 1 - working */
  uint8_t   frontHealth : 1; /*!< Front sensor flag: 0 - not working, 1 - working */
  uint8_t   rightHealth : 1; /*!< Right sensor flag: 0 - not working, 1 - working */
  uint8_t   backHealth : 1;  /*!< Back sensor flag: 0 - not working, 1 - working */
  uint8_t   leftHealth : 1;  /*!< Left sensor flag: 0 - not working, 1 - working */
  uint8_t   upHealth : 1;    /*!< Up sensor health flag: 0 - not working, 1 - working */
  uint8_t   reserved : 2;    /*!< Reserved sensor health flag*/
} osdk_relativePosition; // pack(1)

/*!
 * @brief Timestamp for GPS and RTK
 *
 * @note: Data and time are GMT+8
 */
typedef struct
{
  uint32_t date;     /*!< yyyymmdd E.g.20150205 means February 5th,2015 (GMT+8)*/
  uint32_t time;     /*!< hhmmss E.g. 90209 means 09:02:09 (GMT+8)*/
} osdk_positionTimeStamp; // pack(1)

/*!
 * @brief struct for TOPIC_RTK_POSITION and sub struct for RTK of data broadcast
 */
typedef struct
{
  uint8_t WTF[16]; //We do not need that shit!
  float HFSL;      /*!< height above mean sea level (m) */
} osdk_positionData;        // pack(1)

/*!
 * @brief struct for TOPIC_GPS_DETAILS and sub struct for GPSInfo of data
 * broadcast
 *
 * @note only work outside of simulation
 */
typedef struct
{
  float hdop;       /*!< horizontal dilution of precision */
  float pdop;       /*!< position dilution of precision */
  float fix;        /*!< the state of GPS fix */
  float gnssStatus; /*!< vertical position accuracy (mm) */
  float hacc;       /*!< horizontal position accuracy (mm) */
  float sacc;       /*!< the speed accuracy (cm/s) */
  uint32_t  usedGPS;    /*!< the number of GPS satellites used for pos fix */
  uint32_t  usedGLN; /*!< the number of GLONASS satellites used for pos fix */
  uint16_t  NSV;     /*!< the total number of satellites used for pos fix */
  uint16_t  GPScounter; /*!< the accumulated times of sending GPS data  */
} osdk_GPSDetail;            // pack(1)

/*!
 * @brief struct for GPSInfo of data broadcast
 *
 * @note only work outside of simulation
 */
typedef struct
{
  osdk_positionTimeStamp time;
  int32_t           longitude;   /*!< 1/1.0e7deg */
  int32_t           latitude;    /*!< 1/1.0e7deg */
  int32_t           HFSL;        /*!< height above mean sea level (mm) */
  osdk_vector3f          velocityNED; /*!< cm/s */
  osdk_GPSDetail         detail;
} osdk_GPSInfo; // pack(1)

/*!
 * @brief struct for data broadcast, return magnetometer reading
 *
 * @note returned value is calibrated mag data,
 * 1000 < |mag| < 2000 for normal operation
 */
typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} osdk_mag; // pack(1)

/*!
 * @brief struct for data broadcast and data subscription, return RC reading
 */
typedef struct
{
  int16_t roll;     /*!< [-10000,10000] */
  int16_t pitch;    /*!< [-10000,10000] */
  int16_t yaw;      /*!< [-10000,10000] */
  int16_t throttle; /*!< [-10000,10000] */
  int16_t mode;     /*!< [-10000,10000] */
                    /*!< M100 [P: -8000, A: 0, F: 8000] */
  int16_t gear;     /*!< [-10000,10000] */
                    /*!< M100 [Up: -10000, Down: -4545] */
} osdk_RC;               // pack(1)

/*!
 * @brief struct for TOPIC_GIMBAL_STATUS
 */
typedef struct
{
  uint32_t mountStatus : 1; /*!< 1 - gimbal mounted, 0 - gimbal not mounted*/
  uint32_t isBusy : 1;
  uint32_t pitchLimited : 1;           /*!< 1 - axis reached limit, 0 - no */
  uint32_t rollLimited : 1;            /*!< 1 - axis reached limit, 0 - no */
  uint32_t yawLimited : 1;             /*!< 1 - axis reached limit, 0 - no */
  uint32_t calibrating : 1;            /*!< 1 - calibrating, 0 - no */
  uint32_t prevCalibrationgResult : 1; /*!< 1 - success, 0 - fail */
  uint32_t installedDirection : 1;     /*!< 1 - reversed for OSMO, 0 - normal */
  uint32_t disabled_mvo : 1;
  uint32_t gear_show_unable : 1;
  uint32_t gyroFalut : 1;       /*!< 1 - fault, 0 - normal */
  uint32_t escPitchStatus : 1;  /*!< 1 - Pitch data is normal, 0 - fault */
  uint32_t escRollStatus : 1;   /*!< 1 - Roll data is normal, 0 - fault */
  uint32_t escYawStatus : 1;    /*!< 1 - Yaw data is normal , 0 - fault */
  uint32_t droneDataRecv : 1;   /*!< 1 - normal , 0 - at fault */
  uint32_t initUnfinished : 1;  /*!< 1 - init complete, 0 - not complete */
  uint32_t FWUpdating : 1;      /*!< 1 - updating, 0 - not updating */
  uint32_t reserved2 : 15;
} osdk_gimbalStatus; // pack(1)

/*!
 * @brief struct for data broadcast, return gimbal angle
 */
typedef struct
{

  float roll;           /*!< degree */
  float pitch;          /*!< degree */
  float yaw;            /*!< degree */
  uint8_t   pitchLimit : 1; /*!< 1 - axis reached limit, 0 - no */
  uint8_t   rollLimit : 1;  /*!< 1 - axis reached limit, 0 - no */
  uint8_t   yawLimit : 1;   /*!< 1 - axis reached limit, 0 - no */
  uint8_t   reserved : 5;
} osdk_gimbal; // pack(1)

/*!
 * @brief struct for data broadcast, return flight status
 */
typedef struct
{
  uint8_t flight; /*!<  See FlightStatus/M100FlightStatus in dji_status.hpp */
  uint8_t mode;   /*!<  enum MODE */
  uint8_t gear;   /*!<  See LandingGearMode in dji_status.hpp */
  uint8_t error;  /*!<  enum DJI_ERROR_CODE */
} osdk_status;         // pack(1)

/*!
 * @brief struct for TOPIC_BATTERY_INFO and data broadcast, return battery
 * status
 */
typedef struct
{
  uint32_t capacity;
  int32_t  voltage;
  int32_t  current;
  uint8_t  percentage;
} osdk_battery; // pack(1)

/*!
 * @brief struct for TOPIC_CONTROL_DEVICE and data broadcast, return SDK info
 */
typedef struct
{
  uint8_t controlMode;      /*!< See CtlrMode in dji_status.hpp*/
  uint8_t deviceStatus : 3; /*!< 0->rc  1->app  2->serial*/
  uint8_t flightStatus : 1; /*!< 1->opensd  0->close */
  uint8_t vrcStatus : 1;
  uint8_t reserved : 3;
} osdk_Info; // pack(1)

#endif

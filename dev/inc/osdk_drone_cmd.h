#ifndef _OSDK_DRONE_CMD_H_
#define _OSDK_DRONE_CMD_H_

#define DRONE_CMD_FREQ   200U

#define OSDK_APP_ID   1053960

#define OSDK_VIRTUALRC_SET        0x05
#define OSDK_VIRTUALRC_REQ_ID     0x00
#define OSDK_VIRTUALRC_DATA_ID    0x01

#define OSDK_CTRL_CMD_SET         0x01
#define OSDK_OBTAIN_CTRL_ID       0x00
#define OSDK_MOVEMENT_CTRL_ID     0x03
#define OSDK_ARM_CMD_ID           0x05   //NOTE: USE WITH PRECAUTION TO AVOID GETTING KILLED

#define OSDK_RC_MAX                    10000.0f
#define OSDK_RC_MIN                   -10000.0f
#define OSDK_CTRL_HORI_ATTI_MAX           30.0f     // degree
#define OSDK_CTRL_HORI_ATTI_MIN          -30.0f
#define OSDK_CTRL_HORI_VEL_MAX             4.0f     // m/s
#define OSDK_CTRL_HORI_VEL_MIN            -4.0f
#define OSDK_CTRL_VERT_VEL_MAX            10.0f     // m/s
#define OSDK_CTRL_VERT_VEL_MIN           -10.0f
#define OSDK_CTRL_YAW_RATE_MAX           100.0f     // degree/s
#define OSDK_CTRL_YAW_RATE_MIN          -100.0f

#define OSDK_RC_MODE_P_SDK                10000
#define OSDK_RC_MODE_P                        0
#define OSDK_RC_MODE_A                   -10000
#define OSDK_RC_MODE_DUMMY        (int16_t)(-1)

#define OSDK_FLIGHT_MODE_HORI_ATTI   0b00000000
#define OSDK_FLIGHT_MODE_HORI_VEL    0b01000000
#define OSDK_FLIGHT_MODE_HORI_POS    0b10000000
#define OSDK_FLIGHT_MODE_VERT_VEL    0b00000000
#define OSDK_FLIGHT_MODE_VERT_POS    0b00010000
#define OSDK_FLIGHT_MODE_VERT_THRUST 0b00100000
#define OSDK_FLIGHT_MODE_YAW_ANG     0b00000000
#define OSDK_FLIGHT_MODE_YAW_RATE    0b00001000

#define OSDK_FLIGHT_MODE_GND_FRAME   0b00000000
#define OSDK_FLIGHT_MODE_BODY_FRAME  0b00000010

#define OSDK_FLIGHT_MODE_NON_STABLE  0b00000000
#define OSDK_FLIGHT_MODE_STABLE      0b00000001

typedef struct
{
  uint8_t enable : 1;
  uint8_t cutoff : 1;
  uint8_t reserved : 6;
} __attribute__((packed)) VirtualRC_Setting;

/*!
 * @brief Virtual RC data (supported only on Matrice 100)
 */
typedef struct
{
  //! @note this is default mapping data structure for
  //! virtual remote controller.
  //! @todo channel mapping
  uint32_t roll;
  uint32_t pitch;
  uint32_t throttle;
  uint32_t yaw;
  uint32_t gear;
  uint32_t reserved;
  uint32_t mode;
  uint32_t Channel_07;
  uint32_t Channel_08;
  uint32_t Channel_09;
  uint32_t Channel_10;
  uint32_t Channel_11;
  uint32_t Channel_12;
  uint32_t Channel_13;
  uint32_t Channel_14;
  uint32_t Channel_15;
} __attribute__((packed)) VirtualRC_Data;

typedef struct
{
  uint8_t flag;  /*!< control data flag consists of 8 bits.

                    - CtrlData.flag = ( DJI::OSDK::Control::HorizontalLogic |
                    DJI::OSDK::Control::VerticalLogic |
                    DJI::OSDK::Control::YawLogic |
                    DJI::OSDK::Control::HorizontalCoordinate |
                    DJI::OSDK::Control::StableMode)
                 */
  float x;   /*!< Control with respect to the x axis of the
                    DJI::OSDK::Control::HorizontalCoordinate.*/
  float y;   /*!< Control with respect to the y axis of the
                    DJI::OSDK::Control::HorizontalCoordinate.*/
  float z;   /*!< Control with respect to the z axis, up is positive. */
  float yaw; /*!< Yaw position/velocity control w.r.t. the ground frame.*/

  /*!
   * \brief CtrlData initialize the CtrlData variable.
   * \param in_flag   See CtrlData.flag
   * \param in_x      See CtrlData.x
   * \param in_y      See CtrlData.y
   * \param in_z      See CtrlData.z
   * \param in_yaw    See CtrlData.yaw
   */
} __attribute__((packed)) CtrlData_t;

#ifdef __cplusplus
extern "C" {
#endif

void droneCmd_init(void);
uint16_t droneCmd_activate(uint32_t app_id);
void droneCmd_Flight_control(const uint8_t ctrl_mode,
                                    const float x,
                                    const float y,
                                    const float z,
                                    const float yaw);

#ifdef __cplusplus
}
#endif

#endif

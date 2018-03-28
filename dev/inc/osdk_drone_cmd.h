#ifndef _OSDK_DRONE_CMD_H_
#define _OSDK_DRONE_CMD_H_

#define DRONE_CMD_FREQ   50U

#define OSDK_APP_ID 1053960

#define OSDK_VIRTUALRC_SET        0x05
#define OSDK_VIRTUALRC_REQ_ID     0x00
#define OSDK_VIRTUALRC_DATA_ID    0x01

#define OSDK_ACTIVATION_SET        0x00
#define OSDK_ACTIVATION_ID         0x01
#define OSDK_ACTIVATION_KEY        0x03010A00

typedef struct
{
  uint8_t enable : 1;
  uint8_t cutoff : 1;
  uint8_t reserved : 6;
} VirtualRC_Setting;

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
} VirtualRC_Data;

#ifdef __cplusplus
extern "C" {
#endif

void droneCmd_init(void);
uint16_t droneCmd_activate(uint32_t app_id);

#ifdef __cplusplus
}
#endif

#endif

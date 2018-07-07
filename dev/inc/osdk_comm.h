#ifndef _OSDK_COMM_H_
#define _OSDK_COMM_H_

#include "osdk_protocol_sim.h"

#define  UART_OSDK     &UARTD3
#define  UART_OSDK_BR   115200

#define OSDK_APP_ID   0x4655434B

typedef enum
{
  OSDK_TX_NOWAIT = 0,
  OSDK_TX_WAIT,
} osdk_wait_tx_t;

typedef enum
{
  OSDK_RXCHN_STATE_UNINIT = 0,
  OSDK_RXCHN_STATE_UNSTABLE,
  OSDK_RXCHN_STATE_CONNECTING,
  OSDK_RXCHN_STATE_STABLE,
} osdk_rx_channel_state_t; ///< The state machine for the comm parser

typedef enum
{
  OSDK_PARSE_STATE_UNINIT = 0,
  OSDK_PARSE_STATE_IDLE,
  OSDK_PARSE_STATE_GOT_STX,
  OSDK_PARSE_STATE_GOT_LENGTH,
  OSDK_PARSE_STATE_GOT_TYPE,
  OSDK_PARSE_STATE_GOT_CRC16,
  OSDK_PARSE_STATE_GOT_BAD_CRC16,
  OSDK_PARSE_STATE_GOT_CMD_SET,
  OSDK_PARSE_STATE_GOT_CMD_ID,
  OSDK_PARSE_STATE_GOT_CMD_VAL,
  OSDK_PARSE_STATE_GOT_ACK_VAL,
  OSDK_PARSE_STATE_GOT_CRC32,
  OSDK_PARSE_STATE_GOT_BAD_CRC32
} osdk_parse_state_t; ///< The state machine for the comm parser

typedef enum
{
  OSDK_FRAME_TYPE_INVALID=0,
  OSDK_FRAME_TYPE_CMD,
  OSDK_FRAME_TYPE_ACK,
} osdk_frame_type_t; ///< The state machine for the comm parser

typedef struct
{
  osdk_frame_type_t                type;
  osdk_rx_channel_state_t   rxchn_state;
  osdk_parse_state_t        parse_state;  ///< Parsing state machine
  osdk_frame_t*                rx_frame;
} osdkComm_t;

#ifdef __cplusplus
extern "C" {
#endif

osdkComm_t* osdkComm_get(void);
void osdkComm_init(void);
uint16_t osdk_activate(uint32_t app_id);

uint8_t osdk_StartTX_NoACK(uint8_t* data,
                           const uint8_t        data_len,
                           const uint8_t        cmd_set,
                           const uint8_t        cmd_id,
                           const osdk_wait_tx_t wait);

uint16_t osdk_StartTX_ACK(uint8_t* data,
                          const uint8_t        data_len,
                          const uint8_t        cmd_set,
                          const uint8_t        cmd_id,
                          systime_t timeout);

osdk_timeStamp*    osdk_timeStamp_subscribe(void);
bool               osdk_timeStamp_check(void);

osdk_quaternion*   osdk_attitude_subscribe(void);
bool               osdk_attitude_check(void);
float              osdk_attitude_get_yaw(void);

osdk_RC*           osdk_RC_subscribe(void);
bool               osdk_RC_check(void);

/* Private */
void _osdk_topic_decode(const osdk_flight_data_t* const flight_data);

#ifdef __cplusplus
}
#endif

#endif

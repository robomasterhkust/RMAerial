#include "ch.h"
#include "hal.h"

#include "osdk_crc.h"
#include "osdk_comm.h"
#include <string.h>

static osdkComm_t comm;
static thread_reference_t osdk_receive_thread_handler = NULL;
static thread_reference_t osdk_ack_thread_handler = NULL;
static uint16_t rx_ack = (uint32_t)(-1);
static uint8_t rxbuf[OSDK_MAX_PACKET_LEN * 5];
static uint32_t rxFrame_cnt = 0;

static systime_t last_byte;
static systime_t start_time;
static systime_t txEnd_time = 0;
static systime_t init_wait_time;

static uint32_t tx_seq = 0;
static osdk_frame_t txframe;

osdkComm_t* osdkComm_get(void)
{
  return &comm;
}

static uint16_t osdk_crc16Update(uint16_t crc, uint8_t ch)
{
  uint16_t tmp;
  uint16_t msg;

  msg = 0x00ff & (uint16_t)(ch);
  tmp = crc ^ msg;
  crc = (crc >> 8) ^ crc_tab16[tmp & 0xff];

  return crc;
}

static uint32_t osdk_crc32Update(uint32_t crc, uint8_t ch)
{
  uint32_t tmp;  static uint32_t tx_seq = 0;
  uint32_t msg;

  msg = 0x000000ffL & (uint32_t)(ch);
  tmp = crc ^ msg;
  crc = (crc >> 8) ^ crc_tab32[tmp & 0xff];
  return crc;
}

uint16_t osdk_crc16Calc(const uint8_t* pMsg)
{
  size_t   i;
  uint16_t wCRC = CRC_INIT;

  for (i = 0; i < OSDK_HEADER_LEN - 2; i++)
  {
    wCRC = osdk_crc16Update(wCRC, pMsg[i]);
  }

  return wCRC;
}

uint32_t osdk_crc32Calc(const uint8_t* pMsg, size_t nLen)
{
  size_t   i;
  uint32_t wCRC = CRC_INIT;

  for (i = 0; i < nLen; i++)
  {
    wCRC = osdk_crc32Update(wCRC, pMsg[i]);
  }  static uint32_t tx_seq = 0;

  return wCRC;
}

static void osdkComm_rxchar(void)
{
  USART_TypeDef *u = (*UART_OSDK).usart;
  uint32_t cr1 = u->CR1;

  if(comm.rxchn_state == OSDK_RXCHN_STATE_UNINIT)
  {
    comm.rxchn_state = OSDK_RXCHN_STATE_UNSTABLE;
    goto BYEBYE;
  }

  if(u->DR == OSDK_STX)
  {
    switch(comm.rxchn_state)
    {
      case OSDK_RXCHN_STATE_STABLE:
        //if(ST2US(chVTGetSystemTimeX() - last_byte) > 1000)
        //{
          u->CR1 = cr1 & (~USART_CR1_RXNEIE);
          start_time = chVTGetSystemTimeX();

          if(osdk_receive_thread_handler != NULL)
          {
            chSysLockFromISR();
            chThdResumeI(&osdk_receive_thread_handler,MSG_OK);
            chSysUnlockFromISR();

            osdk_receive_thread_handler = NULL;
          }
      //  }
        break;
      case OSDK_RXCHN_STATE_UNSTABLE:
        if(ST2US(chVTGetSystemTimeX() - last_byte) > 1000)
          comm.rxchn_state = OSDK_RXCHN_STATE_CONNECTING;
        break;
    }
  }
  else if(comm.rxchn_state == OSDK_RXCHN_STATE_CONNECTING)
  {
    if(u->DR > OSDK_MAX_PACKET_LEN_USER)
    {
      comm.rxchn_state = OSDK_RXCHN_STATE_UNSTABLE;
      goto BYEBYE;
    }

    init_wait_time = chVTGetSystemTimeX() + US2ST((u->DR - 1)*1e7/UART_OSDK_BR + 3000);
    u->CR1 = cr1 & (~USART_CR1_RXNEIE);

    chSysLockFromISR();
    chThdResumeI(&osdk_receive_thread_handler,MSG_OK);
    chSysUnlockFromISR();

    osdk_receive_thread_handler = NULL;
  }

  BYEBYE:
  last_byte = chVTGetSystemTimeX();
}

CH_IRQ_HANDLER(STM32_USART3_HANDLER)
{
  CH_IRQ_PROLOGUE();
  osdkComm_rxchar();
  CH_IRQ_EPILOGUE();
}

/**
  * @brief Wait for completion of last uart transmission
  */
static void osdk_waitTX(void)
{
  chThdSleepUntil(txEnd_time);
}

uint8_t osdk_StartTX_NoACK(uint8_t* data,
                           const uint8_t        data_len,
                           const uint8_t        cmd_set,
                           const uint8_t        cmd_id,
                           const osdk_wait_tx_t wait)
{
  if(chVTGetSystemTimeX() < txEnd_time) //Last transmission is not completed
  {
    if(wait)
      osdk_waitTX();
    else
      return 1;
  }

  uint8_t data_frame_len = data_len + 2;

  uint8_t* txframe_p = (uint8_t*)&txframe;
  memset(txframe_p,  0 , OSDK_HEADER_LEN);
  txframe.start = OSDK_STX;
  txframe.len   = OSDK_HEADER_LEN + data_frame_len + 4;
  txframe.seq   = tx_seq++;
  txframe.crc16 = osdk_crc16Calc(txframe_p);

  txframe.data[0] = cmd_set;
  txframe.data[1] = cmd_id;
  memcpy(txframe_p + OSDK_HEADER_LEN + 2, data, data_len);

  uint32_t crc32 = osdk_crc32Calc(txframe_p, OSDK_HEADER_LEN + data_frame_len);
  *((uint32_t*)(&(txframe.data[data_frame_len]))) = crc32; //Put CRC32 in place

  systime_t tx_start_time = chVTGetSystemTimeX();

  uartStartSend(UART_OSDK, txframe.len, txframe_p);

  txEnd_time = tx_start_time + US2ST((txframe.len)*1e7/UART_OSDK_BR + 800);

  return 0;
}

uint16_t osdk_StartTX_ACK(uint8_t* data,
                           const uint8_t        data_len,
                           const uint8_t        cmd_set,
                           const uint8_t        cmd_id,
                          systime_t timeout)
{
  if(chVTGetSystemTimeX() < txEnd_time) //Last transmission is not completed
    osdk_waitTX();

  uint8_t data_frame_len = data_len + 2;

  uint8_t* txframe_p = (uint8_t*)&txframe;
  memset(txframe_p,  0 , OSDK_HEADER_LEN);
  txframe.start = OSDK_STX;
  txframe.len   = OSDK_HEADER_LEN + data_frame_len + 4;
  txframe.seq   = tx_seq++;
  txframe.session   =  OSDK_SESSION_REALLY_NEED_ACK;        //Need ack
  txframe.crc16 = osdk_crc16Calc(txframe_p);

  txframe.data[0] = cmd_set;
  txframe.data[1] = cmd_id;
  memcpy(txframe_p + OSDK_HEADER_LEN + 2, data, data_len);

  uint32_t crc32 = osdk_crc32Calc(txframe_p, OSDK_HEADER_LEN + data_frame_len);
  *((uint32_t*)(&(txframe.data[data_frame_len]))) = crc32; //Put CRC32 in place

  systime_t tx_start_time = chVTGetSystemTimeX();

  uartStartSend(UART_OSDK, txframe.len, txframe_p);

  txEnd_time = tx_start_time + US2ST((txframe.len)*1e7/UART_OSDK_BR + 800);
/*
  chSysLock();
  msg_t rxmsg = chThdSuspendTimeoutS(&osdk_ack_thread_handler, timeout);
  chSysUnlock();

  if(rxmsg = MSG_TIMEOUT)
    return (uint16_t)(-1);

  uint16_t result = rx_ack;
  rx_ack = (uint16_t)(-1);
  return result;*/

  return 0;
}

//static void rxend_error(UARTDriver *uartp)
//{
//  comm.rxchn_state = OSDK_RXCHN_STATE_UNSTABLE;
//}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
  NULL,NULL,NULL,NULL,NULL,
  UART_OSDK_BR,
  0,
  0,
  0
};

static inline void osdk_startRX(uint8_t *const buf, const uint16_t len)
{
  uartStopReceive(UART_OSDK);
  uartStartReceive(UART_OSDK, OSDK_MAX_PACKET_LEN * 5, buf);
  (*UART_OSDK).usart->CR1 |= USART_CR1_RXNEIE;

  chSysLock();
  chThdSuspendS(&osdk_receive_thread_handler);
  chSysUnlock();
}

static uint8_t osdk_get_start(void)
{
  (*UART_OSDK).usart->CR1 |= USART_CR1_RXNEIE;
  chSysLock();
  chThdSuspendS(&osdk_receive_thread_handler);
  chSysUnlock();

  chThdSleepUntil(init_wait_time);
  comm.rxchn_state = OSDK_RXCHN_STATE_STABLE;
}

uint16_t osdk_activate(uint32_t app_id)
{
  char activation_string[] = "12345678901234567890123456789012";
                           //"00000000000000000000000000000000"
                           //"FW: 127646/0LSDK-v1.1 BETA A3-03.02.36.13/0";
  activation_string[6] = 0x00;
  activation_string[35] = 0x00;

  uint8_t activation_seq[44];
  *((uint32_t*)activation_seq)       = app_id;
  *((uint32_t*)(activation_seq + 4)) = (uint32_t)2;
  *((uint32_t*)(activation_seq + 8)) = OSDK_ACTIVATION_KEY;
  memcpy(activation_seq + 12, activation_string, 32);

  //uint16_t result = osdk_StartTX_ACK(activation_seq, 44,
  //  OSDK_ACTIVATION_SET, OSDK_ACTIVATION_ID, MS2ST(1000));
  //return result;

  osdk_StartTX_NoACK(activation_seq, 44,
    OSDK_ACTIVATION_SET, OSDK_ACTIVATION_ID, OSDK_TX_WAIT);
  return 0;
}


static THD_WORKING_AREA(osdk_rx_wa, 4096);
static THD_FUNCTION(osdk_rx, p)
{
  (void)p;
  chRegSetThreadName("DJI OSDK receiver");

  osdk_get_start(); //Discard the first packet, as it may be incorrect
  while(!chThdShouldTerminateX())
  {
      osdk_startRX(rxbuf, OSDK_MAX_PACKET_LEN);

      comm.rx_frame = (osdk_frame_t*)rxbuf;

      while(!comm.rx_frame->len)
        chThdSleepMicroseconds(100);

      systime_t end_time = start_time + US2ST((comm.rx_frame->len)*1e7/UART_OSDK_BR + 200);
      /*
        Transimission time estimation: bytes to transfer *
        (8bits per byte + 1 start bit + 1 stop bit) on UART / UART baudrate + 200us in case we missed the last bit
      */

      if(end_time > chVTGetSystemTimeX())
        chThdSleepUntil(end_time);

      uint32_t crc32 = *(uint32_t*)(rxbuf + comm.rx_frame->len - 4);
      if(crc32 == osdk_crc32Calc(rxbuf, comm.rx_frame->len - 4))
      {
        if(comm.rx_frame->data[0] == OSDK_PUSH_DATA_SET &&
           comm.rx_frame->data[1] == OSDK_FLIGHT_DATA_ID)
        {
          rxFrame_cnt++;
          _osdk_topic_decode((osdk_flight_data_t*)(comm.rx_frame->data));
        }
        else if(comm.rx_frame->ack && osdk_ack_thread_handler != NULL)
        {
          rx_ack = *((uint16_t*)(&comm.rx_frame->data));
          if(osdk_ack_thread_handler == NULL)
          {
            chThdResume(&osdk_ack_thread_handler, MSG_OK);
            osdk_ack_thread_handler = NULL;
          }
        }
      }

      chThdSleepMilliseconds(2); //Discard unused frames

      //Found a following packet just after the previous one, Fuck it!! Discard it!!

      if(rxbuf[comm.rx_frame->len] == OSDK_STX)
      {
        comm.rx_frame = (osdk_frame_t*)(&rxbuf[comm.rx_frame->len]);
        end_time += US2ST((comm.rx_frame->len)*1e7/UART_OSDK_BR + 200);
        if(end_time > chVTGetSystemTimeX())
          chThdSleepUntil(end_time);
      }

      rxbuf[0] = rxbuf[1] = 0;
  }
}

void osdkComm_init(void)
{
  memset(&comm,0,sizeof(osdkComm_t));
  uartStart(UART_OSDK, &uart_cfg);

  chThdCreateStatic(osdk_rx_wa, sizeof(osdk_rx_wa), NORMALPRIO + 4, osdk_rx ,NULL);
}

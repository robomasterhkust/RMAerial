#include "ch.h"
#include "hal.h"

#include "osdk_comm.h"
#include <string.h>

static osdkComm_t comm;
static thread_reference_t osdk_receive_thread_handler = NULL;
static uint8_t rxbuf[OSDK_MAX_PACKET_LEN];

static systime_t last_byte;
static systime_t start_time;
static uint32_t  init_wait_time;

osdkComm_t* osdkComm_get(void)
{
  return &comm;
}

static void osdkComm_rxchar(void)
{
  USART_TypeDef *u = (*UART_OSDK).usart;
  uint32_t cr1 = u->CR1;

  if(u->DR == OSDK_STX
    || rxbuf[0] == OSDK_STX)
  {
    switch(comm.rxchn_state)
    {
      case OSDK_RXCHN_STATE_STABLE:
        u->CR1 = cr1 & (~USART_CR1_RXNEIE);
        start_time = chVTGetSystemTimeX();
        comm.parse_state = OSDK_PARSE_STATE_GOT_STX;

        if(osdk_receive_thread_handler != NULL)
        {
          chSysLockFromISR();
          chThdResumeI(&osdk_receive_thread_handler,MSG_OK);
          chSysUnlockFromISR();

          osdk_receive_thread_handler = NULL;
        }
        break;
      case OSDK_RXCHN_STATE_UNSTABLE:
        if(ST2US(chVTGetSystemTimeX() - last_byte) > 500)
          comm.rxchn_state = OSDK_RXCHN_STATE_CONNECTING;
        break;
    }
  }
  else if(comm.rxchn_state == OSDK_RXCHN_STATE_CONNECTING)
  {
    init_wait_time = chVTGetSystemTimeX() + US2ST((u->DR - 1)*1e7/UART_OSDK_BR) + 100;
    u->CR1 = cr1 & (~USART_CR1_RXNEIE);

    chSysLockFromISR();
    chThdResumeI(&osdk_receive_thread_handler,MSG_OK);
    chSysUnlockFromISR();

    osdk_receive_thread_handler = NULL;
  }
  else if(comm.rxchn_state == OSDK_RXCHN_STATE_UNINIT)
    comm.rxchn_state = OSDK_RXCHN_STATE_UNSTABLE;

  last_byte = chVTGetSystemTimeX();
}

CH_IRQ_HANDLER(STM32_USART3_HANDLER)
{
  CH_IRQ_PROLOGUE();
  osdkComm_rxchar();
  CH_IRQ_EPILOGUE();
}

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
  uartStartReceive(UART_OSDK, len, buf);
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

static THD_WORKING_AREA(osdk_rx_wa, 1024);
static THD_FUNCTION(osdk_rx, p)
{
  (void)p;
  chRegSetThreadName("DJI OSDK receiver");

  osdk_get_start(); //Deal with the first packet, as it may be incorrect

  comm.rx_frame = (osdk_frame_t*)rxbuf;
  comm.parse_state = OSDK_PARSE_STATE_IDLE;

  while(!chThdShouldTerminateX())
  {
    osdk_startRX(rxbuf, OSDK_MAX_PACKET_LEN);

    while(!comm.rx_frame->len)
      chThdSleepMicroseconds(100);

    systime_t end_time = start_time + US2ST((comm.rx_frame->len)*1e7/UART_OSDK_BR + 100);
    /*
      Transimission time estimation: bytes to transfer *
      (8bits per byte + 1 start bit + 1 stop bit) on UART / UART baudrate + 20us in case we missed the last bit
    */

    if(end_time > chVTGetSystemTimeX())
      chThdSleepUntil(end_time);

    if(comm.rx_frame->data[0] == OSDK_PUSH_DATA_SET &&
       comm.rx_frame->data[1] == OSDK_FLIGHT_DATA_ID)
       _osdk_topic_decode((osdk_flight_data_t*)comm.rx_frame->data);

    rxbuf[0] = rxbuf[1] = 0;
  }
}

void osdkComm_init(void)
{
  memset(&comm,0,sizeof(osdkComm_t));

  uartStart(UART_OSDK, &uart_cfg);
  chThdCreateStatic(osdk_rx_wa, sizeof(osdk_rx_wa), NORMALPRIO, osdk_rx ,NULL);
}

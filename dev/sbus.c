/**
 * Edward ZHANG, Terry ZENG, 20180304
 * @file    sbus.c
 * @brief   Dbus driver and decoder with keyboard and mouse support and safe lock
 */

#include "ch.h"
#include "hal.h"

#include "string.h"
#include "sbus.h"

static uint8_t rxbuf[SBUS_BUFFER_SIZE];
static SBUS_t RC_Ctl;
static thread_reference_t uart_sbus_thread_handler = NULL;
static uint8_t rx_start_flag = 1;
static SBUS_state_t rc_state = SBUS_UNCONNECTED;

SBUS_state_t SBUS_getState(void)
{
  return rc_state;
}

/**
 * @brief   Return the RC_Ctl struct
 */
SBUS_t* SBUS_get(void)
{
  return &RC_Ctl;
}

#define SBUS_START_BYTE 0x0F
#define SBUS_END_BYTE   0x00

/**
 * @brief Reset RC controller
 * @NOTE  This function is also used as safe lock mechanism for RC controller
 *        S2 is not flushed because it is used to unlock the RC controller
 */
static void RC_reset(void)
{
  RC_Ctl.ch1 = SBUS_CH_VALUE_OFFSET;
  RC_Ctl.ch2 = SBUS_CH_VALUE_OFFSET;
  RC_Ctl.ch3 = SBUS_CH_VALUE_OFFSET;
  RC_Ctl.ch4 = SBUS_CH_VALUE_OFFSET;
}

/**
 * @briSBUS_START_BYTEef   Decode the received SBUS sequence and store it in RC_Ctl struct
 */
static void decryptSBUS(void)
{
  memcpy(&RC_Ctl, rxbuf, SBUS_BUFFER_SIZE);

  if(RC_Ctl.start != SBUS_START_BYTE && RC_Ctl.end != SBUS_END_BYTE)
  {
    RC_reset();
    rc_state = SBUS_UNCONNECTED;
  }
  else if(RC_Ctl.failsafe || RC_Ctl.frame_lost)
  {
    RC_reset();
    rc_state = SBUS_OFFLINE;
  }
  else
    rc_state = SBUS_CONNECTED;
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp)
{
  chSysLockFromISR();
  chThdResumeI(&uart_sbus_thread_handler, MSG_OK);
  uart_sbus_thread_handler = NULL;
  chSysUnlockFromISR();
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
  NULL,NULL,rxend,NULL,NULL,
  100000,
  USART_CR1_PCE,
  USART_CR2_STOP_1,
  0,
  0
};

#define  SBUS_INIT_WAIT_TIME_MS      6U
#define  SBUS_WAIT_TIME_MS         100U
static THD_WORKING_AREA(uart_sbus_thread_wa, 512);
static THD_FUNCTION(uart_sbus_thread, p)
{
  (void)p;
  chRegSetThreadName("uart sbus receiver");

  uartStart(UART_SBUS, &uart_cfg);
  dmaStreamRelease(*UART_SBUS.dmatx);
  msg_t rxmsg;
  systime_t timeout = MS2ST(SBUS_INIT_WAIT_TIME_MS);

  while(!chThdShouldTerminateX())
  {
    uartStopReceive(UART_SBUS);
    uartStartReceive(UART_SBUS, SBUS_BUFFER_SIZE, rxbuf);

    chSysLock();
    rxmsg = chThdSuspendTimeoutS(&uart_sbus_thread_handler, timeout);
    chSysUnlock();

    if(rxmsg == MSG_OK)
    {
      chSysLock();
      decryptSBUS();
      chSysUnlock();
    }
    else
      RC_reset();

    if(rc_state != SBUS_UNCONNECTED)
      timeout = MS2ST(SBUS_WAIT_TIME_MS);
    else
      timeout = MS2ST(SBUS_INIT_WAIT_TIME_MS);
  }
}

/**
 * @brief   Initialize the RC receiver
 */
void SBUS_init(void)
{
  RC_reset();
  chThdCreateStatic(uart_sbus_thread_wa, sizeof(uart_sbus_thread_wa),
                    NORMALPRIO + 7,
                    uart_sbus_thread, NULL);
}

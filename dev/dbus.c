/**
 * Edward ZHANG, Terry ZENG, 20180304
 * @file    dbus.c
 * @brief   Dbus driver and decoder with keyboard and mouse support and safe lock
 */

#include "ch.h"
#include "hal.h"

#include "dbus.h"

static RC_Ctl_t RC_pilot, RC_gimbal;

/**
 * @brief   Decode the received DBUS sequence and store it in RC_pilot struct
 */
static void decryptDBUS(RC_Ctl_t* rc, const uint8_t* rxbuf)
{
  rc->rc.channel0 = ((rxbuf[0]) | (rxbuf[1]<<8)) & 0x07FF;
  rc->rc.channel1 = ((rxbuf[1]>>3) | (rxbuf[2]<<5)) & 0x07FF;
  rc->rc.channel2 = ((rxbuf[2]>>6) | (rxbuf[3]<<2) | ((uint32_t)rxbuf[4]<<10)) & 0x07FF;
  rc->rc.channel3 = ((rxbuf[4]>>1) | (rxbuf[5]<<7)) & 0x07FF;
  rc->rc.s1  = ((rxbuf[5] >> 4)& 0x000C) >> 2;                         //!< Switch left
  rc->rc.s2  = ((rxbuf[5] >> 4)& 0x0003);


  rc->mouse.x = rxbuf[6] | (rxbuf[7] << 8);                   //!< Mouse X axis
  rc->mouse.y = rxbuf[8] | (rxbuf[9] << 8);                   //!< Mouse Y axis
  rc->mouse.z = rxbuf[10] | (rxbuf[11] << 8);                 //!< Mouse Z axis
  rc->mouse.LEFT = rxbuf[12];                                       //!< Mouse Left Is Press ?
  rc->mouse.RIGHT = rxbuf[13];                                       //!< Mouse Right Is Press ?
  rc->keyboard.key_code = rxbuf[14] | (rxbuf[15] << 8);                   //!< KeyBoard value
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend_pilot(UARTDriver *uartp) {
  if(uartp == UART_DBUS_PILOT)
  {
    if(RC_pilot.rx_start_flag)
    {
      chSysLockFromISR();
      chThdResumeI(&(RC_pilot.thread_handler), MSG_OK);
      chSysUnlockFromISR();
    }
    else
      RC_pilot.rx_start_flag = 1;
  }
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend_gimbal(UARTDriver *uartp) {
  if(uartp == UART_DBUS_GIMBAL)
  {
    if(RC_gimbal.rx_start_flag)
    {
      chSysLockFromISR();
      chThdResumeI(&(RC_gimbal.thread_handler), MSG_OK);
      chSysUnlockFromISR();
    }
    else
      RC_gimbal.rx_start_flag = 1;
  }
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_pilot_cfg = {
  NULL,NULL,rxend_pilot,NULL,NULL,
  100000,
  USART_CR1_PCE,
  0,
  0
};

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_gimbal_cfg = {
  NULL,NULL,rxend_gimbal,NULL,NULL,
  100000,
  USART_CR1_PCE,
  0,
  0
};

/**
 * @brief   Return the RC_pilot struct
 */
RC_Ctl_t* RC_get(const rc_index_t index)
{
  if(index == RC_INDEX_PILOT)
    return &RC_pilot;
  else if(index == RC_INDEX_GIMBAL)
    return &RC_gimbal;
}

/**
 * @brief Reset RC controller
 * @NOTE  This function is also used as safe lock mechanism for RC controller
 *        S2 is not flushed because it is used to unlock the RC controller
 */
static void RC_RCreset(RC_Ctl_t* rc)
{
  rc->rc.channel0 = 1024;
  rc->rc.channel1 = 1024;
  rc->rc.channel2 = 1024;
  rc->rc.channel3 = 1024;

}

static void RC_reset(RC_Ctl_t* rc)
{
  RC_RCreset(rc);

  rc->rc.s1 =0;
  rc->rc.s2 = 0;
  rc->mouse.LEFT=0;
  rc->mouse.RIGHT =0;
  rc->mouse.x=0;
  rc->mouse.y=0;
  rc->mouse.z=0;
  rc->keyboard.key_code=0;
}

#define  DBUS_INIT_WAIT_TIME_MS      4U
#define  DBUS_WAIT_TIME_MS         100U
static THD_WORKING_AREA(uart_dbus_pilot_thread_wa, 512);
static THD_WORKING_AREA(uart_dbus_gimbal_thread_wa, 512);
static THD_FUNCTION(uart_dbus_thread, p)
{
  RC_Ctl_t* rc = (RC_Ctl_t*)p;
  chRegSetThreadName("uart dbus receiver");

  msg_t rxmsg;
  systime_t timeout = MS2ST(DBUS_INIT_WAIT_TIME_MS);

  rc->state = RC_STATE_LOST;

  while(!chThdShouldTerminateX())
  {
    uartStopReceive(rc->uart);
    uartStartReceive(rc->uart, DBUS_BUFFER_SIZE, rc->rxbuf);

    chSysLock();
    rxmsg = chThdSuspendTimeoutS(&(rc->thread_handler), timeout);
    chSysUnlock();

    if(rxmsg == MSG_OK)
    {
      if(rc->state == RC_STATE_LOST)
      {
        timeout = MS2ST(DBUS_WAIT_TIME_MS);
        rc->state = RC_STATE_CONNECTED;
      }
      else
      {
        chSysLock();
        decryptDBUS(rc, rc->rxbuf);
        chSysUnlock();
      }
    }
    else
    {
      rc->state = RC_STATE_LOST;//rxflag = false;
      RC_reset(rc);
      timeout = MS2ST(DBUS_INIT_WAIT_TIME_MS);
    }
  }
}

/**
 * @brief   Initialize the RC receiver
 */
void RC_init(void)
{
  memset(&RC_pilot, 0 ,sizeof(RC_Ctl_t));
  memset(&RC_gimbal, 0 ,sizeof(RC_Ctl_t));

  RC_pilot.uart = UART_DBUS_PILOT;
  RC_pilot.thread_handler = NULL;

  RC_gimbal.uart = UART_DBUS_GIMBAL;
  RC_gimbal.thread_handler = NULL;

  uartStart(UART_DBUS_PILOT, &uart_pilot_cfg);
  dmaStreamRelease(*UART_DBUS_PILOT.dmatx);
  uartStart(UART_DBUS_GIMBAL, &uart_gimbal_cfg);
  dmaStreamRelease(*UART_DBUS_GIMBAL.dmatx);

  RC_reset(&RC_pilot);
  RC_reset(&RC_gimbal);

  chThdCreateStatic(uart_dbus_pilot_thread_wa, sizeof(uart_dbus_pilot_thread_wa),
                    NORMALPRIO + 7,
                    uart_dbus_thread, &RC_pilot);
  chThdCreateStatic(uart_dbus_gimbal_thread_wa, sizeof(uart_dbus_gimbal_thread_wa),
                    NORMALPRIO + 7,
                    uart_dbus_thread, &RC_gimbal);
}

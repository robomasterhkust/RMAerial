#include "dji_hard_driver.hpp"
#include "ch.h"
#include "hal.h"

#define UART_DJI_A3 &UARTD6

class DJI_bridge //: public DJI::OSDK::HardDriver
{
public:
  virtual void init(){}

  virtual size_t send(const uint8_t* buf, size_t len)
  {
    uartStopSend(UART_DJI_A3);
    uartStartSend(UART_DJI_A3, len, buf);

    return 0;
  }

  virtual DJI::OSDK::time_ms getTimeStamp()
  {
    return ST2MS(chVTGetSystemTimeX());
  }

  virtual bool getDeviceStatus()
  {
    return true;
  }

  virtual size_t readall(uint8_t* buf, size_t maxlen)
  {
    return 8;
  }

  static void delay_nms(uint16_t time)
  {
    chThdSleepMilliseconds(time);
  }
};

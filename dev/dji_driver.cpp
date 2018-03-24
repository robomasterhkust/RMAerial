#include "ch.h"
#include "hal.h"

#include "dji_driver.hpp"
#include "Activate.h"

using namespace DJI::OSDK;

uint8_t DJI_driver::start()
{
  if (black->protocolLayer->getDriver()->getDeviceStatus())
  {
    delete (black);
    return -1;
  }

  chThdSleepMilliseconds(60);

  black->functionalSetUp();
  chThdSleepMilliseconds(500);

  if (black->getFwVersion() > 0 &&
    black->getFwVersion() < extendedVersionBase &&
    black->getFwVersion() != Version::M100_31)
  {
    delete (black);
    return -2;
  }

  userActivate();
  chThdSleepMilliseconds(500);
  /*ACK::ErrorCode ack = waitForACK();
  if(ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, func);
  }*/

  // Verify subscription
  if (black->getFwVersion() != Version::M100_31)
  {
    black->subscribe->verify();
    chThdSleepMilliseconds(500);
  }

  // Obtain Control Authority
  black->obtainCtrlAuthority();
  chThdSleepMilliseconds(1000);
}

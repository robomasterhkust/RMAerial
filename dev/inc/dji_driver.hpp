#include "dji_vehicle.hpp"

namespace DJI
{
namespace OSDK
{

class DJI_driver{
protected:
  Vehicle* black; //The name of our drone

public:
  DJI_driver(void)
  {
    black = new Vehicle(false);
  }

  uint8_t start();
};

}}

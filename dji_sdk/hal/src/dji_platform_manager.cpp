/*! @file dji_platform_manager.cpp
 *  @version 3.4
 *  @date Dec 2017
 *
 *  @brief Data protection and thread management abstract classes.
 *
 *  @Copyright (c) 2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "dji_platform_manager.hpp"

using namespace DJI::OSDK;

PlatformManager::PlatformManager()
{
}

PlatformManager::~PlatformManager()
{
}

Thread*
PlatformManager::addThread(Vehicle* vehicle_ptr, uint8_t thread_type)
{
  //! Threads not supported by default
  return NULL;
}

HardDriver*
PlatformManager::addHardDriver(uint8_t driver_type, const char* device_port,
                               uint32_t baudrate)
{
  if (driver_type == PlatformManager::SERIAL_DEVICE)
  {
    DJI_bridge* dji = new DJI_bridge;
    return dji;
  }
  else
  {
    return NULL;
  }
}

void
PlatformManager::millisecSleep(int milliseconds)
{
  DJI_bridge::delay_nms(milliseconds);
}

#ifndef COMPONENTS_SPIBUSLOCKS_HPP_
#define COMPONENTS_SPIBUSLOCKS_HPP_

#include "Mutex.hpp"

namespace BusLocks {
extern Mutex spi1;	// GPS (GpsTask) and IMU32A (PollingTask)
extern Mutex spi2;
}

#endif /* COMPONENTS_SPIBUSLOCKS_HPP_ */

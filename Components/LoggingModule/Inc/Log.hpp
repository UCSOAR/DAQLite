/*
 * Log.hpp
 *
 *  Created on: Jan 21, 2026
 *      Author: jaddina
 */

#ifndef LOGGINGMODULE_INC_LOG_HPP_
#define LOGGINGMODULE_INC_LOG_HPP_

#include <cstdint>
#include <cstddef>



enum class LoggingStatus
{
	LOGGING_SUCCESS,
	LOG_HIGHER_PRIORITY,
	LOG_LOWER_PRIORITY,
	LOGGING_ERR,
	LOG_FLASH_READY,
	LOG_FLASH_NOT_READY,
	FLASH_FULL

};

enum class LoggingDest
{
	RAM,
	FLASH_EXTERN,
	FILE_SYSTEM,
	DMA,
};

enum class LoggingData
{
	IMU32G,
	IMU16G,
	MAG,
	BARO07,
	BARO11,
	GPS,
	FILTER,		// altitude prediction filter output (AltitudePredictionFilter/Output/Inc/FilterLog.hpp)
	FILTER2,	// filter diagnostics, 1 Hz
	EVENT		// filter flight event (FilterLog::Event)


};

enum class LoggingPriority
{
	FIFTH = 0,
	FOURTH = 1,
	THIRD = 2,
	SECOND = 3,
	FIRST = 4,
};

struct LoggingPacket{

	LoggingDest dest;
	LoggingData dataType;
	const uint8_t* data;
	uint32_t dataSize;
	LoggingPriority priority;

};


#endif /* LOGGINGMODULE_INC_LOG_HPP_ */

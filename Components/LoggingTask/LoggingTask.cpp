/*
 * LoggingTask.cpp
 *
 *  Created on: Jan 23, 2026
 *      Author: jaddina
 */


/**
 ********************************************************************************
 * @file    LoggingTask.cpp
 * @author  jaddina
 * @date    Sep 13, 2025
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "LoggingTask.hpp"
#include "SystemDefines.hpp"
#include "Command.hpp"
#include "LoggingService.hpp"

#include "DataBroker.hpp"
#include "Task.hpp"
#include "DebugTask.hpp"

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/************************************
 * VARIABLES
 ************************************/

/************************************
 * FUNCTION DECLARATIONS
 ************************************/


uint8_t LoggingTask::buf[20] = {0};
bool LoggingTask::highAltitude = false;
bool LoggingTask::lowAltitude = false;

//bool LoggingTask::firstAlt = false;

float firstAlt = 0.0f;
bool firstAltCaptured = false;
bool launched = false;
bool landed = false;



/************************************
 * FUNCTION DEFINITIONS
 ************************************/
LoggingTask::LoggingTask():Task(TASK_LOGGING_QUEUE_DEPTH_OBJS)
{

}



/**
 * @brief Initialize the LoggingTask
 *        Do not modify this function aside from adding the task name
 */
void LoggingTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize watchdog task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)LoggingTask::RunTask,
            (const char*)"LoggingTask",
            (uint16_t)TASK_LOGGING_QUEUE_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)TASK_LOGGING_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

                SOAR_ASSERT(rtValue == pdPASS, "LoggingTask::InitTask() - xTaskCreate() failed");

	DataBroker::Subscribe<IMUData>(this);
	DataBroker::Subscribe<BaroData>(this);
	DataBroker::Subscribe<MagData>(this);
	DataBroker::Subscribe<GPSData>(this);

}

void LoggingTask::Run(void * pvParams){



    while (1) {
        /* Process commands in blocking mode */
        Command cm;
        bool res = qEvtQueue->ReceiveWait(cm);
        if(res){

        	HandleCommand(cm);
        }
    }

}

void LoggingTask::HandleCommand(Command& cm){

	// SOAR_PRINT("Data In Logging Task\n");
	if(cm.GetCommand() != DATA_BROKER_COMMAND){
		return;
	}

	LoggingStatus err = LoggingStatus::LOGGING_SUCCESS;
	DataBrokerMessageTypes messageType = DataBroker::getMessageType(cm);

	switch(messageType){

	case DataBrokerMessageTypes::IMU_DATA:
	{
		IMUData data = DataBroker::ExtractData<IMUData>(cm);
		uint32_t timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

		// Only log IDs 0 or 1
		if (data.id != 0 && data.id != 1) {
			cm.Reset();
		    return;
		}

//
		// Prepare buffer for flash logging
		buf[0] = static_cast<uint8_t>(data.id == 0 ? LoggingData::IMU16G : LoggingData::IMU32G);

		// Manual serialization to avoid padding issues
		memcpy(buf + 1, &timestamp, sizeof(timestamp));        // 4 bytes timestamp
		memcpy(buf + 5, &data.accel, sizeof(data.accel));     // 6 bytes accel
		memcpy(buf + 11, &data.gyro, sizeof(data.gyro));      // 6 bytes gyro
		memcpy(buf + 17, &data.temp, sizeof(data.temp));      // 2 bytes temp
		buf[19] = data.id;                                     // 1 byte id

		// Log to flash
		LoggingService log(
		    LoggingDest::FLASH_EXTERN,
		    data.id == 0 ? LoggingData::IMU16G : LoggingData::IMU32G,
		    buf,
		    20,  // total bytes: 1(type) + 4(timestamp) + 6+6+2+1 = 20
		    LoggingPriority::SECOND
		);
		err = log.LogData();
		break;
	}

	case DataBrokerMessageTypes:: MAG_DATA:
	{
		MagData data = DataBroker::ExtractData<MagData>(cm);

		uint32_t timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

		//SOAR_PRINT("Mag data (%d, %d, %d)\n", data.magX, data.magY, data.magZ);
		memset(buf, 0, 20);

		buf[0] = static_cast<uint8_t>(LoggingData::MAG);

		//add timestamp 4 bytes
		memcpy(buf + 1, &timestamp, sizeof(timestamp));

		//add mag XYZ 12 bytes
		memcpy(buf + 5, &data.magX, sizeof(data.magX));
		memcpy(buf + 9, &data.magY, sizeof(data.magY));
		memcpy(buf + 13, &data.magZ, sizeof(data.magZ));

		memset(buf + 17, 0, 20 - 1 - 16); // pad rest with zero


		LoggingService log = LoggingService(LoggingDest::FLASH_EXTERN, LoggingData::MAG, buf, 20, LoggingPriority::SECOND);
		err = log.LogData();

		break;
	}

	case DataBrokerMessageTypes:: BARO_DATA:
	{
		BaroData data = DataBroker::ExtractData<BaroData>(cm);

		// Get timestamp in ms
		uint32_t timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;


		// Determine BARO type
		buf[0] = static_cast<uint8_t>(data.id == 0 ? LoggingData::BARO07 : LoggingData::BARO11);

		// Copy timestamp (4 bytes)
		memcpy(buf + 1, &timestamp, sizeof(timestamp));

		// Copy pressure (4 bytes) and temperature (2 bytes)
		memcpy(buf + 5, &data.pressure, sizeof(data.pressure));
		memcpy(buf + 9, &data.temp, sizeof(data.temp));

		// Pad remaining bytes with zeros so the total slot is 20 bytes
		memset(buf + 11, 0, 20 - 1 - 10); // 1 byte type + 10 bytes data already, last byte is ID

		// Set ID as the last byte of the 20-byte slot
		buf[19] = data.id;

		// Log the full 20-byte sample
		LoggingService log(
		    LoggingDest::FLASH_EXTERN,
		    data.id == 0 ? LoggingData::BARO07 : LoggingData::BARO11,
		    buf,
		    20,                  // full 20-byte slot
		    LoggingPriority::SECOND
		);

		err = log.LogData();

		break;
	}

	case DataBrokerMessageTypes:: GPS_DATA:
	{
		//gets gps data from data broker
		GPSData data = DataBroker::ExtractData<GPSData>(cm);
		//get timestamp for the log
		const uint32_t timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

		//manually counts the length of the NMEA sentence
		uint32_t sentenceLen = 0;
		while ((sentenceLen < sizeof(data.buffer_)) && (data.buffer_[sentenceLen] != '\0'))
		{
			sentenceLen++;
		}

		//breaks if there is no sentence
		if (sentenceLen == 0)
		{
			break;
		}

		//storing 12 bytes per 20 byte slot
		constexpr uint8_t kGpsPayloadBytes = 12;
		//total chunks/slots required is the ceiling division of sentenceLen and GpsPayload sze in bytes
		const uint8_t chunkCount = static_cast<uint8_t>((sentenceLen + kGpsPayloadBytes - 1U) / kGpsPayloadBytes); //ceiling division

		//loop through each chunk and copy the appropriate bits into each 20 byte slot
		//number of slots required for the message is the number chunks
		for (uint8_t chunkIdx = 0; chunkIdx < chunkCount; chunkIdx++)
		{
			//set the buffer to all 0
			memset(buf, 0, sizeof(buf));

			//calculate sentence offset (byte that needs to be copied into slot)
			const uint32_t offset = static_cast<uint32_t>(chunkIdx) * kGpsPayloadBytes;
			//calculate the payLoadLen for this chunk, if the sentenceLen - offset is greater then the 12 byte max of GPS data per slot,
			//then as usual max out the buffer with 12 bytes, otherwise just fill the buffer with the remainder which could be less than 12 bytes
			//(this would be a paritally ful slot)
			const uint8_t payloadLen = static_cast<uint8_t>(((sentenceLen - offset) > kGpsPayloadBytes) ? kGpsPayloadBytes : (sentenceLen - offset));

			//fill the buffer with all the necessary data
			buf[0] = static_cast<uint8_t>(LoggingData::GPS);
			memcpy(buf + 1, &timestamp, sizeof(timestamp));
			buf[5] = chunkIdx;
			buf[6] = chunkCount;
			buf[7] = payloadLen;
			memcpy(buf + 8, data.buffer_ + offset, payloadLen);

			LoggingService log(
			    LoggingDest::FLASH_EXTERN,
			    LoggingData::GPS,
			    buf,
			    20,
			    LoggingPriority::SECOND
			);

			//Log the data to flash
			err = log.LogData();
			if (err == LoggingStatus::LOGGING_ERR)
			{
				break;
			}
		}

		break;
	}
	case DataBrokerMessageTypes :: INVALID:
	{
		cm.Reset();
		return;
	}

	}

	if(err == LoggingStatus::LOGGING_ERR){
		cm.Reset();
		SOAR_PRINT("Log was unsuccessful\n");
		return;
	}
	else if(err == LoggingStatus::LOG_FLASH_NOT_READY || err == LoggingStatus::LOG_FLASH_READY){
		cm.Reset();
		return;
	}

	cm.Reset();

	return;

}



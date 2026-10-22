/*
 * PollingTask.cpp
 *
 *  Created on: Apr 8, 2026
 *      Author: jaddina
 */

#include "PollingTask.hpp"
#include "FreeRTOS.h"
#include "timers.h"
#include "DataBroker.hpp"
#include "LoggingService.hpp"




PollingTask::PollingTask():Task(TASK_LOGGING_QUEUE_DEPTH_OBJS), pollTimerHandle(nullptr), imu16(), imu32a(), imu32b(), magnetometer()
{

}

/**
 * @brief Initialize the PollingTask
 *        Do not modify this function aside from adding the task name
 */
void PollingTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize watchdog task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)PollingTask::RunTask,
            (const char*)"PollingTask",
            (uint16_t)TASK_LOGGING_QUEUE_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)TASK_LOGGING_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

                SOAR_ASSERT(rtValue == pdPASS, "PollingTask::InitTask() - xTaskCreate() failed");


	imu16.Init(hspi3_, LSM6DSO_CS_PIN,  LSM6DSO_CS_PORT );
    imu32a.Init(hspi1_, LSM6DSO32_CS_PORT, LSM6DSO32_CS_PIN);
	imu32b.Init(hspi4_, LSM6DSO32_CS_PORT, LSM6DSO32_CS_PIN);
	magnetometer.Init(hspi2_, MMC_CS_PORT, MMC_CS_PIN);
	gpsInitialized = gps.Init(hspi1_, GPS_CS_PORT, GPS_CS_PIN);

	pollTimerHandle = xTimerCreate("PollTimer", pdMS_TO_TICKS(pollPeriodMs), pdTRUE, (void*)this, PollingTask::PollTimerCallback);
	SOAR_ASSERT(pollTimerHandle != nullptr, "PollingTask::InitTask() - xTimerCreate() failed");

	// Timer is started later by the START_POLLING command (debug "poll")
}


void PollingTask::Run(void * pvParams){


    while (1) {

        Command cm;
		bool res = qEvtQueue->ReceiveWait(cm);
		if(res){

        	HandleCommand(cm);
        }
    }
}

void PollingTask::HandleCommand(Command& cm){
	switch(cm.GetCommand()){
	case DATA_COMMAND:
		HandleRequestCommand(cm.GetTaskCommand());
		break;

	case TASK_SPECIFIC_COMMAND:
		break;
	}

	cm.Reset();

}

void PollingTask::HandleRequestCommand(uint16_t taskCommand){
	switch(taskCommand){
	case POLL_SENSORS_AND_LOG:
		PollSensors();
		LogData();
		break;
	case START_POLLING:
		if (xTimerIsTimerActive(pollTimerHandle) != pdFALSE) {
			SOAR_PRINT("PollingTask - Already polling\n");
			break;
		}
		pollingTimerStarted = false;
		if (xTimerStart(pollTimerHandle, 0) == pdPASS) {
			SOAR_PRINT("PollingTask - Polling started (%d ms period)\n", pollPeriodMs);
		} else {
			SOAR_PRINT("PollingTask - Failed to start poll timer\n");
		}
		break;
	case GPS_TEST:
		if (!gpsInitialized)
		{
			SOAR_PRINT("GPS TEST | GPS not initialized\n");
		}
		else if (!gps.getGGALine(gpsData.buffer_))
		{
			SOAR_PRINT("GPS TEST | Timed out waiting for GGA sentence\n");
		}
		else
		{
			SOAR_PRINT("GPS RAW | %s\n", gpsData.buffer_);
			GPSData rawGps = gpsData;
			DataBroker::Publish<GPSData>(&rawGps);
			gps.ParseGpsData(&gpsData);
			SOAR_PRINT("GPS GGA | time=%d lat_deg=%d lat_min=%d lon_deg=%d lon_min=%d antAlt=%d antUnit=%d geoidAlt=%d geoidUnit=%d totalAlt=%d totalUnit=%d\n",
					   (int32_t)gpsData.time_,
					   (int32_t)gpsData.latitude_.degrees_,
					   (int32_t)gpsData.latitude_.minutes_,
					   (int32_t)gpsData.longitude_.degrees_,
					   (int32_t)gpsData.longitude_.minutes_,
					   (int32_t)gpsData.antennaAltitude_.altitude_,
					   (int32_t)gpsData.antennaAltitude_.unit_,
					   (int32_t)gpsData.geoidAltitude_.altitude_,
					   (int32_t)gpsData.geoidAltitude_.unit_,
					   (int32_t)gpsData.totalAltitude_.altitude_,
					   (int32_t)gpsData.totalAltitude_.unit_);
		}
		break;
	case IMU32_A_TEST: {
		uint8_t data[14] = {0};
		imu32a.ReadSensors(data);
		imu32DataA = imu32a.ConvertRawMeasurementToStruct(data);
		imu32DataA.id = 1;

		SOAR_PRINT("IMU32A | id=%u accel=(%d,%d,%d) gyro=(%d,%d,%d) temp=%d\n",
				   (unsigned int)imu32DataA.id,
				   (int)imu32DataA.accel.x,
				   (int)imu32DataA.accel.y,
				   (int)imu32DataA.accel.z,
				   (int)imu32DataA.gyro.x,
				   (int)imu32DataA.gyro.y,
				   (int)imu32DataA.gyro.z,
				   (int)imu32DataA.temp);
		break;
	}
	case IMU32_B_TEST: {
		uint8_t data[14] = {0};
		imu32b.ReadSensors(data);
		imu32DataB = imu32b.ConvertRawMeasurementToStruct(data);
		imu32DataB.id = 1;

		SOAR_PRINT("IMU32B | id=%u accel=(%d,%d,%d) gyro=(%d,%d,%d) temp=%d\n",
				   (unsigned int)imu32DataB.id,
				   (int)imu32DataB.accel.x,
				   (int)imu32DataB.accel.y,
				   (int)imu32DataB.accel.z,
				   (int)imu32DataB.gyro.x,
				   (int)imu32DataB.gyro.y,
				   (int)imu32DataB.gyro.z,
				   (int)imu32DataB.temp);
		break;
	}
	case IMU16_TEST: {
		uint8_t data[14] = {0};
		imu16.readSensors(data);
		imu16Data = imu16.bytesToStruct(data, true, true, true);
		imu16Data.id = 0;

		SOAR_PRINT("IMU16 | id=%u accel=(%d,%d,%d) gyro=(%d,%d,%d) temp=%d\n",
				   (unsigned int)imu16Data.id,
				   (int)imu16Data.accel.x,
				   (int)imu16Data.accel.y,
				   (int)imu16Data.accel.z,
				   (int)imu16Data.gyro.x,
				   (int)imu16Data.gyro.y,
				   (int)imu16Data.gyro.z,
				   (int)imu16Data.temp);
		break;
	}
	case BARO07_TEST:
		baro07Data = barometer07.getSample();
		baro07Data.id = 0;

		SOAR_PRINT("BARO07 | id=%u pressure=%d temp=%d\n",
				(unsigned int)baro07Data.id,
				(int)baro07Data.pressure,
				(int)baro07Data.temp);
		break;
	case BARO11_TEST:
		baro11Data = barometer11.getSample();
		baro11Data.id = 0;

		SOAR_PRINT("BARO11 | id=%u pressure=%d temp=%d\n",
				(unsigned int)baro11Data.id,
				(int)baro11Data.pressure,
				(int)baro11Data.temp);
		break;
	case MAG_TEST:
		magnetometer.triggerMeasurement();
		magnetometer.readData(driverData);

		magData.magX = driverData.scaledX;
		magData.magY = driverData.scaledY;
		magData.magZ = driverData.scaledZ;

		SOAR_PRINT("MAG | id%u magX%d magY%d madZ%d\n",
				(int)magData.magX,
				(int)magData.magY,
				(int)magData.magZ
		);
	default:
		break;
	}
}

void PollingTask::PollTimerCallback(TimerHandle_t xTimer)
{
	PollingTask* task = static_cast<PollingTask*>(pvTimerGetTimerID(xTimer));
	if ((task == nullptr) || (task->GetEventQueue() == nullptr)) {
		return;
	}

	Command cmd(DATA_COMMAND, PollingTask::POLL_SENSORS_AND_LOG);
	task->GetEventQueue()->Send(cmd, true);

}


void PollingTask::PollSensors()
{
	uint8_t data[14] = {0};

	baro07Data = barometer07.getSample();
	baro07Data.id = 0;

	baro11Data = barometer11.getSample();
	baro11Data.id = 1;

	imu16.readSensors(data);
	imu16Data = imu16.bytesToStruct(data, true, true, true);
	imu16Data.id = 0;

	imu32b.ReadSensors(data);
	imu32DataB = imu32b.ConvertRawMeasurementToStruct(data);
	imu32DataB.id = 1;

	imu32a.ReadSensors(data);
	imu32DataA = imu32a.ConvertRawMeasurementToStruct(data);
	imu32DataA.id = 1;

	magnetometer.triggerMeasurement();
	magnetometer.readData(driverData);

	magData.magX = driverData.scaledX;
	magData.magY = driverData.scaledY;
	magData.magZ = driverData.scaledZ;

	static const char kMockGga[83] ="$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*59\r\n";
	memset(gpsData.buffer_, 0, sizeof(gpsData.buffer_));
	strncpy(gpsData.buffer_, kMockGga, sizeof(gpsData.buffer_) - 1);
	GPSData parsedGps = gpsData;
	gps.ParseGpsData(&parsedGps);


}


void PollingTask::LogData(){
	TickType_t nowTick = xTaskGetTickCount();

	if(!pollingTimerStarted){
		pollingStartTick = nowTick;
		previousLogTick = nowTick;
		pollingTimerStarted = true;
	}

	TickType_t elapsedTicks = nowTick - pollingStartTick;
	TickType_t deltaTicks   = nowTick - previousLogTick;

	float elapsedSec = (float)elapsedTicks / configTICK_RATE_HZ;
	float deltaSec   = (float)deltaTicks / configTICK_RATE_HZ;
	uint32_t rateHz     = (deltaSec > 0.0f) ? (1.0f / deltaSec) : 0.0f;

	uint32_t elapsedMs = (elapsedTicks * 1000U) / configTICK_RATE_HZ;
	uint32_t deltaMs   = (deltaTicks * 1000U) / configTICK_RATE_HZ;

	SOAR_PRINT("PollingTask Timing | elapsed=%d ms dt=%d ms rate=%d Hz\n",
			   elapsedMs,
			   deltaMs,
			   rateHz);

	previousLogTick = nowTick;

	SOAR_PRINT("Log data polled");

	DataBroker::Publish<IMUData>(&imu16Data);
	DataBroker::Publish<IMUData>(&imu32DataB);
	DataBroker::Publish<IMUData>(&imu32DataA);
	DataBroker::Publish<BaroData>(&baro07Data);
	DataBroker::Publish<BaroData>(&baro11Data);
	DataBroker::Publish<MagData>(&magData);
	DataBroker::Publish<GPSData>(&gpsData);
}

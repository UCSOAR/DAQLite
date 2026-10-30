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
#include "SPIBusLocks.hpp"




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
            (UBaseType_t)TASK_POLLING_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

                SOAR_ASSERT(rtValue == pdPASS, "PollingTask::InitTask() - xTaskCreate() failed");


	imu16.Init(hspi3_, LSM6DSO_CS_PIN,  LSM6DSO_CS_PORT );
    imu32a.Init(hspi1_, LSM6DSO32_CS_PORT, LSM6DSO32_CS_PIN);
	imu32b.Init(hspi4_, LSM6DSO32_CS_PORT, LSM6DSO32_CS_PIN);
	magnetometer.Init(hspi2_, MMC_CS_PORT, MMC_CS_PIN);
	// GPS is configured and read by GpsTask

	pollTimerHandle = xTimerCreate("PollTimer", pdMS_TO_TICKS(GROUND_POLL_PERIOD_MS), pdTRUE, (void*)this, PollingTask::PollTimerCallback);
	SOAR_ASSERT(pollTimerHandle != nullptr, "PollingTask::InitTask() - xTimerCreate() failed");

	// Timer is started by FlightTask with SET_GROUND_RATE once the scheduler runs
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
	case SET_GROUND_RATE:
		SetPollPeriod(GROUND_POLL_PERIOD_MS);
		break;
	case SET_FLIGHT_RATE:
		SetPollPeriod(FLIGHT_POLL_PERIOD_MS);
		break;
	case IMU32_A_TEST: {
		uint8_t data[14] = {0};
		BusLocks::spi1.Lock();		// SPI1 is shared with the GPS (GpsTask)
		imu32a.ReadSensors(data);
		BusLocks::spi1.Unlock();
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
		imu32DataB.id = 2;

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


/**
 * @brief Changes the poll timer period, also starts the timer if it is stopped
 */
void PollingTask::SetPollPeriod(uint32_t periodMs)
{
	if (xTimerChangePeriod(pollTimerHandle, pdMS_TO_TICKS(periodMs), 0) == pdPASS) {
		SOAR_PRINT("PollingTask - rate set to %u ms\n", (unsigned int)periodMs);
	} else {
		SOAR_PRINT("PollingTask - failed to set rate\n");
	}
}


void PollingTask::PollSensors()
{
	const TickType_t startTick = xTaskGetTickCount();
	uint8_t data[14] = {0};

	baro07Data = barometer07.getSample();
	baro07Data.id = 0;

	// baro11 and imu32b are not polled (baro11 drifts ~1000 Pa on a still board). Both can still
	// be read with the baro11 / imu32b debug commands

	imu16.readSensors(data);
	imu16Data = imu16.bytesToStruct(data, true, true, true);
	imu16Data.id = 0;

	BusLocks::spi1.Lock();		// SPI1 is shared with the GPS (GpsTask)
	imu32a.ReadSensors(data);
	BusLocks::spi1.Unlock();
	imu32DataA = imu32a.ConvertRawMeasurementToStruct(data);
	imu32DataA.id = 1;

	magnetometer.triggerMeasurement();
	magnetometer.readData(driverData);

	magData.magX = driverData.scaledX;
	magData.magY = driverData.scaledY;
	magData.magZ = driverData.scaledZ;

	// GPS: GpsTask publishes the real GGA sentences as they arrive (no more mock sentence every poll)

	lastPollDurationMs = (xTaskGetTickCount() - startTick) * portTICK_PERIOD_MS;
}


void PollingTask::LogData(){
	DataBroker::Publish<IMUData>(&imu16Data);
	DataBroker::Publish<IMUData>(&imu32DataA);
	DataBroker::Publish<BaroData>(&baro07Data);
	DataBroker::Publish<MagData>(&magData);
}

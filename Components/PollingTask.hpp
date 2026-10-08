/*
 * PollingTask.hpp
 *
 *  Created on: Apr 8, 2026
 *      Author: jaddina
 */
#ifndef COMPONENTS_POLLINGTASK_HPP_
#define COMPONENTS_POLLINGTASK_HPP_

#include "Task.hpp"
#include "MS5607Driver.hpp"
#include "MS5611Driver.hpp"
#include "lsm6dso.hpp"
#include <LSM6DO32Driver.h>
#include "mmc5983ma.hpp"
#include "NEO-M9N-00BDriver.hpp"
#include "SensorDataTypes.hpp"
#include "main.h"



extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;



class PollingTask: public Task
{
	public:

		enum PollingTaskCommands : uint16_t {
			POLL_SENSORS_AND_LOG = 1,
			GPS_TEST,
		};

		static PollingTask& Inst() {
			static PollingTask inst;
			return inst;
		}

		void InitTask();

		// Sets how often (in milliseconds) PollSensors()/LogData() run. Call before InitTask().
		void SetPollPeriodMs(uint32_t periodMs) { pollPeriodMs = periodMs; }

	protected:

		static void RunTask(void* pvParams) { PollingTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
		void Run(void * pvParams); // Main run code
		void HandleCommand(Command& cm);
		void HandleRequestCommand(uint16_t taskCommand);
		static void PollTimerCallback(TimerHandle_t xTimer);



	private:
		// Private Functions
		PollingTask();        // Private constructor
		PollingTask(const PollingTask&);                        // Prevent copy-construction
		PollingTask& operator=(const PollingTask&);				// Prevent assignment
		void LogData();
		void PollSensors();

		TimerHandle_t pollTimerHandle;
		uint32_t pollPeriodMs = 100;    // How often PollSensors()/LogData() run, set via SetPollPeriodMs()

		//to count time elapsed and time taken per poll and Hz
		TickType_t pollingStartTick = 0;
		TickType_t previousLogTick = 0;
		bool pollingTimerStarted = false;
		bool gpsInitialized = false;
		bool canNetworkReady = false;
		TickType_t lastCanJoinRetryTick = 0;

		//sensor data structs
		BaroData baro07Data;
		BaroData baro11Data;
		IMUData imu32Data;
		IMUData imu16Data;
		MagData magData;
		MagDriverData driverData;
		GPSData gpsData;

		//All sensor gpios and spi
		//Baro 07 config
		GPIO_TypeDef* MS5607_CS_PORT = BARO07_CS_GPIO_Port;
		const uint16_t MS5607_CS_PIN = BARO07_CS_Pin;
		SPI_HandleTypeDef* hspi3_= &hspi3;

		//Baro 11 config
		GPIO_TypeDef* MS5611_CS_PORT = BARO11_CS_GPIO_Port;
		const uint16_t MS5611_CS_PIN = BARO11_CS_Pin;
		SPI_HandleTypeDef* hspi1_ = &hspi1;

		//IMU 32 config
		GPIO_TypeDef *LSM6DSO32_CS_PORT = IMU32_CS_GPIO_Port;
		const uint16_t LSM6DSO32_CS_PIN = IMU32_CS_Pin;
		SPI_HandleTypeDef *hspi2_ = &hspi2;

		//IMU 16 config
		GPIO_TypeDef *LSM6DSO_CS_PORT = IMU16_CS_GPIO_Port;
		const uint16_t LSM6DSO_CS_PIN = IMU16_CS_Pin;


		//Mag config
	    SPI_HandleTypeDef *hspi4_ = &hspi4;
	    GPIO_TypeDef *MMC_CS_PORT = MAG_CS_GPIO_Port;
	    const uint16_t MMC_CS_PIN = MAG_CS_Pin;

	    //GPS config
	    GPIO_TypeDef *GPS_CS_PORT = GPS_CS_GPIO_Port;
	   	const uint16_t GPS_CS_PIN = GPS_CS_Pin;


	    //Drivers
	    LSM6DSO_Driver imu16;
	    LSM6DO32_Driver imu32;
	    MS5607_Driver barometer07{hspi2_, MS5607_CS_PORT, MS5607_CS_PIN};
	    MS5611_Driver barometer11{hspi4_, MS5611_CS_PORT, MS5611_CS_PIN};
	    MMC5983MA magnetometer;
	    NEOM9N00B gps;

};



#endif /* COMPONENTS_POLLINGTASK_HPP_ */

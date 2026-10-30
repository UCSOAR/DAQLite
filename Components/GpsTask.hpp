/*
 * GpsTask.hpp
 *
 *  The GPS (NEO-M9N on SPI1) runs in its own task: it waits for NMEA sentences without holding up
 *  PollingTask, and publishes each GGA sentence (GPSData) on the DataBroker as it arrives.
 *  LoggingTask logs it like any other sensor message (one sentence a second at the receiver's
 *  default rate). SPI1 is shared with IMU32A, every transfer holds BusLocks::spi1
 */
#ifndef COMPONENTS_GPSTASK_HPP_
#define COMPONENTS_GPSTASK_HPP_

#include "Task.hpp"
#include "NEO-M9N-00BDriver.hpp"
#include "SensorDataTypes.hpp"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

class GpsTask: public Task
{
	public:

		enum GpsTaskCommands : uint16_t {
			GPS_TEST = 1,	// debug "gps": last sentence received and the receive counts
			GPS_INIT		// debug "gps_init": configure the receiver again (not in flight)
		};

		static GpsTask& Inst() {
			static GpsTask inst;
			return inst;
		}

		void InitTask();

	protected:

		static void RunTask(void* pvParams) { GpsTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
		void Run(void* pvParams); // Main run code
		void HandleCommand(Command& cm);

	private:
		GpsTask();        // Private constructor
		GpsTask(const GpsTask&);                        // Prevent copy-construction
		GpsTask& operator=(const GpsTask&);				// Prevent assignment

		void PublishSentence();
		void PrintStatus();
		void Reinit();

		static constexpr uint32_t IDLE_POLL_MS = 5;				// receiver had nothing to send: wait before reading again
		static constexpr uint32_t NOT_INITIALIZED_POLL_MS = 100;	// only debug commands until gps_init works

		NEOM9N00B gps;
		bool gpsInitialized = false;
		char line[NMEA_MAX_LENGTH + 1] = {};	// sentence being collected (NEOM9N00B::pollGGALine)
		GPSData lastGps{};						// last published sentence, parsed (GPS_TEST)
		uint32_t sentences = 0;					// GGA sentences published since boot
		uint32_t lastSentenceMs = 0;

		//GPS config
		GPIO_TypeDef* GPS_CS_PORT = GPS_CS_GPIO_Port;
		const uint16_t GPS_CS_PIN = GPS_CS_Pin;
		SPI_HandleTypeDef* hspi1_ = &hspi1;
};

#endif /* COMPONENTS_GPSTASK_HPP_ */

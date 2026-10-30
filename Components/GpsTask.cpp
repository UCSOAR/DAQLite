/*
 * GpsTask.cpp
 *
 *  GPS in its own task (see GpsTask.hpp)
 */

#include "GpsTask.hpp"
#include "DataBroker.hpp"
#include "FlightTask.hpp"
#include "SPIBusLocks.hpp"
#include <cstring>

GpsTask::GpsTask():Task(TASK_GPS_QUEUE_DEPTH_OBJS)
{

}

/**
 * @brief Initialize the GpsTask
 *        Configures the receiver here, before the scheduler runs, as PollingTask did
 */
void GpsTask::InitTask()
{
	// Make sure the task is not already initialized
	SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize GPS task twice");

	BaseType_t rtValue =
		xTaskCreate((TaskFunction_t)GpsTask::RunTask,
			(const char*)"GpsTask",
			(uint16_t)TASK_GPS_STACK_DEPTH_WORDS,
			(void*)this,
			(UBaseType_t)TASK_GPS_PRIORITY,
			(TaskHandle_t*)&rtTaskHandle);

	SOAR_ASSERT(rtValue == pdPASS, "GpsTask::InitTask() - xTaskCreate() failed");

	gpsInitialized = gps.Init(hspi1_, GPS_CS_PORT, GPS_CS_PIN);
	SOAR_PRINT("GPS INIT | %s\n", gpsInitialized ? "VALSET messages sent" : "failed (gps_init to retry)");
}

/**
 * @brief Reads the receiver in short SPI1 transfers (BusLocks::spi1 held for one 32 byte read
 *        at a time, so IMU32A never waits long), publishes every GGA sentence as it arrives
 */
void GpsTask::Run(void* pvParams)
{
	while (1) {
		// Debug commands, never waits
		Command cm;
		while (qEvtQueue->Receive(cm)) {
			HandleCommand(cm);
		}

		if (!gpsInitialized) {
			osDelay(NOT_INITIALIZED_POLL_MS);
			continue;
		}

		BusLocks::spi1.Lock();
		const bool sentence = gps.pollGGALine(line);
		BusLocks::spi1.Unlock();

		if (sentence) {
			PublishSentence();
		} else if (!gps.lastPollHadData()) {
			osDelay(IDLE_POLL_MS);		// receiver empty, otherwise read the rest straight away
		}
	}
}

void GpsTask::HandleCommand(Command& cm)
{
	if (cm.GetCommand() == TASK_SPECIFIC_COMMAND) {
		switch (cm.GetTaskCommand()) {
		case GPS_TEST:
			PrintStatus();
			break;
		case GPS_INIT:
			Reinit();
			break;
		default:
			break;
		}
	}
	cm.Reset();
}

/**
 * @brief One GGA sentence: published raw for the flash log, kept parsed for GPS_TEST
 */
void GpsTask::PublishSentence()
{
	// Without the trailing \r\n the sentence is one line in the flash dump
	size_t len = strlen(line);
	while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
		line[--len] = '\0';
	}

	GPSData data{};
	strncpy(data.buffer_, line, sizeof(data.buffer_) - 1);
	DataBroker::Publish<GPSData>(&data);

	// ParseGpsData splits the buffer it is given, so it gets its own copy
	lastGps = data;
	gps.ParseGpsData(&lastGps);
	strncpy(lastGps.buffer_, data.buffer_, sizeof(lastGps.buffer_) - 1);
	sentences++;
	lastSentenceMs = TICKS_TO_MS(xTaskGetTickCount());
}

void GpsTask::PrintStatus()
{
	if (!gpsInitialized) {
		SOAR_PRINT("GPS TEST | GPS not initialized (gps_init to retry)\n");
		return;
	}
	if (sentences == 0) {
		SOAR_PRINT("GPS TEST | no GGA sentence received yet\n");
		return;
	}
	const uint32_t ageMs = TICKS_TO_MS(xTaskGetTickCount()) - lastSentenceMs;
	SOAR_PRINT("GPS RAW | %s (%lu sentences, last %lu ms ago)\n", lastGps.buffer_,
			   (unsigned long)sentences, (unsigned long)ageMs);
	SOAR_PRINT("GPS GGA | time=%d lat_deg=%d lat_min=%d lon_deg=%d lon_min=%d antAlt=%d antUnit=%d geoidAlt=%d geoidUnit=%d totalAlt=%d totalUnit=%d\n",
			   (int32_t)lastGps.time_,
			   (int32_t)lastGps.latitude_.degrees_,
			   (int32_t)lastGps.latitude_.minutes_,
			   (int32_t)lastGps.longitude_.degrees_,
			   (int32_t)lastGps.longitude_.minutes_,
			   (int32_t)lastGps.antennaAltitude_.altitude_,
			   (int32_t)lastGps.antennaAltitude_.unit_,
			   (int32_t)lastGps.geoidAltitude_.altitude_,
			   (int32_t)lastGps.geoidAltitude_.unit_,
			   (int32_t)lastGps.totalAltitude_.altitude_,
			   (int32_t)lastGps.totalAltitude_.unit_);
}

/**
 * @brief gps_init: configuring waits for the receiver's ACKs with SPI1 held (IMU32A blocked for up
 *        to a few seconds), so it is refused in flight
 */
void GpsTask::Reinit()
{
	if (FlightTask::Inst().GetFlightState() == FlightState::FLIGHT) {
		SOAR_PRINT("GPS INIT | refused in flight (it holds SPI1, which IMU32A needs)\n");
		return;
	}
	BusLocks::spi1.Lock();
	gpsInitialized = gps.Init(hspi1_, GPS_CS_PORT, GPS_CS_PIN);
	BusLocks::spi1.Unlock();
	SOAR_PRINT("GPS INIT | %s\n", gpsInitialized ? "VALSET messages sent" : "failed");
}

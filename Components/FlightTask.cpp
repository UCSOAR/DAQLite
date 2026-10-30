/*
 * FlightTask.cpp
 *
 *  Created on: Oct 23, 2026
 *      Author: jaddina
 */

#include "FlightTask.hpp"
#include "PollingTask.hpp"
#include "DataBroker.hpp"
#include "SensorDataTypes.hpp"


FlightTask::FlightTask():Task(TASK_FLIGHT_QUEUE_DEPTH_OBJS), stateMachine(FlightThresholds::Normal())
{

}

/**
 * @brief Initialize the FlightTask
 *        Runs before the scheduler starts, so no prints in here
 */
void FlightTask::InitTask()
{
	// Make sure the task is not already initialized
	SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize flight task twice");

	BaseType_t rtValue =
		xTaskCreate((TaskFunction_t)FlightTask::RunTask,
			(const char*)"FlightTask",
			(uint16_t)TASK_FLIGHT_STACK_DEPTH_WORDS,
			(void*)this,
			(UBaseType_t)TASK_FLIGHT_PRIORITY,
			(TaskHandle_t*)&rtTaskHandle);

	SOAR_ASSERT(rtValue == pdPASS, "FlightTask::InitTask() - xTaskCreate() failed");

	DataBroker::Subscribe<IMUData>(this);
	DataBroker::Subscribe<BaroData>(this);
}

void FlightTask::Run(void * pvParams){

	// Every boot starts in GROUND, FlightTask owns the polling rate
	Command rateCmd(DATA_COMMAND, PollingTask::SET_GROUND_RATE);
	PollingTask::Inst().GetEventQueue()->Send(rateCmd);

	while (1) {
		Command cm;
		bool res = qEvtQueue->ReceiveWait(cm);
		if(res){
			HandleCommand(cm);
		}
	}
}

void FlightTask::HandleCommand(Command& cm){
	switch(cm.GetCommand()){
	case DATA_BROKER_COMMAND:
		HandleDataBrokerCommand(cm);
		break;
	case TASK_SPECIFIC_COMMAND:
		HandleTaskCommand(cm.GetTaskCommand());
		break;
	default:
		break;
	}

	cm.Reset();
}

void FlightTask::HandleDataBrokerCommand(Command& cm){
	const uint32_t nowMs = NowMs();
	bool changed = false;

	switch(DataBroker::getMessageType(cm)){
	case DataBrokerMessageTypes::IMU_DATA:
	{
		IMUData data = DataBroker::ExtractData<IMUData>(cm);
		if (data.id == IMU32A_ID) {
			changed = stateMachine.OnAccel(data.accel.x, data.accel.y, data.accel.z, nowMs);
		}
		break;
	}
	case DataBrokerMessageTypes::BARO_DATA:
	{
		BaroData data = DataBroker::ExtractData<BaroData>(cm);
		if (data.id == BARO07_ID) {
			changed = stateMachine.OnBaro(data.id, data.pressure, nowMs);
		}
		break;
	}
	default:
		break;
	}

	// force_flight: the state machine keeps running (accel / altitude stay live) but is not
	// allowed to land, put it straight back into FLIGHT without announcing a state change
	if (forceFlight && stateMachine.GetState() != FlightState::FLIGHT) {
		stateMachine.Restore(FlightState::FLIGHT, nowMs);
		changed = false;
	}

	if (changed) {
		OnStateChange(nowMs);
	}

	if (testMode && (nowMs - lastTestPrintMs >= TEST_PRINT_PERIOD_MS)) {
		lastTestPrintMs = nowMs;
		SOAR_PRINT("FLIGHT TEST | %s accel=%u mg alt=%d m\n",
				   stateMachine.GetState() == FlightState::FLIGHT ? "FLIGHT" : "GROUND",
				   (unsigned int)stateMachine.GetLastAccelMg(),
				   (int)stateMachine.GetAltitudeAglM());
	}
}

void FlightTask::HandleTaskCommand(uint16_t taskCommand){
	switch(taskCommand){
	case PRINT_STATE:
		PrintState(NowMs());
		break;
	case RESTORE_FLIGHT:
	case RESTORE_LANDED:
	{
		const bool inFlight = (taskCommand == RESTORE_FLIGHT);
		stateMachine.Restore(inFlight ? FlightState::FLIGHT : FlightState::GROUND, NowMs());
		hasFlown = true;
		flightState = stateMachine.GetState();
		Command rateCmd(DATA_COMMAND, inFlight ? PollingTask::SET_FLIGHT_RATE : PollingTask::SET_GROUND_RATE);
		PollingTask::Inst().GetEventQueue()->Send(rateCmd);
		SOAR_PRINT("FLIGHT: restored %s after a power loss\n", inFlight ? "FLIGHT" : "GROUND (landed)");
		break;
	}
	case TOGGLE_FORCE_FLIGHT:
	{
		forceFlight = !forceFlight;
		if (forceFlight) {
			const bool wasFlight = (stateMachine.GetState() == FlightState::FLIGHT);
			if (!wasFlight) {
				stateMachine.Restore(FlightState::FLIGHT, NowMs());
				OnStateChange(NowMs());	// flight poll rate, hasFlown, prints GROUND -> FLIGHT
			}
			SOAR_PRINT("FlightTask - force flight ON, staying in FLIGHT until force_flight is sent again\n");
		} else {
			SOAR_PRINT("FlightTask - force flight OFF, normal detection (lands after the board is still)\n");
		}
		break;
	}
	case TOGGLE_TEST_MODE:
		testMode = !testMode;
		stateMachine.SetThresholds(testMode ? FlightThresholds::BenchTest() : FlightThresholds::Normal());
		SOAR_PRINT("FlightTask - test mode %s\n", testMode ? "ON (bench thresholds)" : "OFF (flight thresholds)");
		break;
	default:
		break;
	}
}

void FlightTask::OnStateChange(uint32_t nowMs){
	const bool inFlight = (stateMachine.GetState() == FlightState::FLIGHT);
	if (inFlight) {
		hasFlown = true;
	}
	flightState = stateMachine.GetState();

	Command rateCmd(DATA_COMMAND, inFlight ? PollingTask::SET_FLIGHT_RATE : PollingTask::SET_GROUND_RATE);
	PollingTask::Inst().GetEventQueue()->Send(rateCmd);

	SOAR_PRINT("FLIGHT: %s at %u ms (accel=%u mg, alt=%d m)\n",
			   inFlight ? "GROUND -> FLIGHT" : "FLIGHT -> GROUND",
			   (unsigned int)nowMs,
			   (unsigned int)stateMachine.GetLastAccelMg(),
			   (int)stateMachine.GetAltitudeAglM());
}

void FlightTask::PrintState(uint32_t nowMs){
	SOAR_PRINT("FLIGHT STATE | state=%s inState=%u ms accel=%u mg\n",
			   stateMachine.GetState() == FlightState::FLIGHT ? "FLIGHT" : "GROUND",
			   (unsigned int)(nowMs - stateMachine.GetStateEnteredMs()),
			   (unsigned int)stateMachine.GetLastAccelMg());

	if (stateMachine.HasAltitude()) {
		SOAR_PRINT("FLIGHT STATE | alt=%d m testMode=%s forceFlight=%s pollDuration=%u ms\n",
				   (int)stateMachine.GetAltitudeAglM(),
				   testMode ? "ON" : "OFF", forceFlight ? "ON" : "OFF",
				   (unsigned int)PollingTask::Inst().GetLastPollDurationMs());
	} else {
		SOAR_PRINT("FLIGHT STATE | alt=n/a testMode=%s forceFlight=%s pollDuration=%u ms\n",
				   testMode ? "ON" : "OFF", forceFlight ? "ON" : "OFF",
				   (unsigned int)PollingTask::Inst().GetLastPollDurationMs());
	}
}

uint32_t FlightTask::NowMs(){
	return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/*
 * FlightTask.hpp
 *
 *  Created on: Oct 23, 2026
 *      Author: jaddina
 *
 *  Watches DataBroker IMU/baro data, runs the GROUND/FLIGHT state machine and
 *  sets PollingTask's rate. Never touches sensor hardware.
 */
#ifndef COMPONENTS_FLIGHTTASK_HPP_
#define COMPONENTS_FLIGHTTASK_HPP_

#include "Task.hpp"
#include "FlightStateMachine.hpp"

class FlightTask: public Task
{
	public:

		enum FlightTaskCommands : uint16_t {
			PRINT_STATE = 1,
			TOGGLE_TEST_MODE = 2
		};

		static FlightTask& Inst() {
			static FlightTask inst;
			return inst;
		}

		void InitTask();

		// Read by other tasks (SensorFrameTask: pad = GROUND before any flight, landed = GROUND after one)
		FlightState GetFlightState() const { return flightState; }
		bool HasFlown() const { return hasFlown; }

	protected:
		static void RunTask(void* pvParams) { FlightTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
		void Run(void * pvParams); // Main run code
		void HandleCommand(Command& cm);

	private:
		// Private Functions
		FlightTask();        // Private constructor
		FlightTask(const FlightTask&);                        // Prevent copy-construction
		FlightTask& operator=(const FlightTask&);				// Prevent assignment

		void HandleDataBrokerCommand(Command& cm);
		void HandleTaskCommand(uint16_t taskCommand);
		void OnStateChange(uint32_t nowMs);
		void PrintState(uint32_t nowMs);
		static uint32_t NowMs();

		static constexpr uint8_t IMU32A_ID = 1;					// only imu32a is used for flight detection
		static constexpr uint32_t TEST_PRINT_PERIOD_MS = 1000;

		FlightStateMachine stateMachine;
		volatile FlightState flightState = FlightState::GROUND;	// copy of stateMachine's state for other tasks
		volatile bool hasFlown = false;							// set on the first GROUND -> FLIGHT, never cleared
		bool testMode = false;
		uint32_t lastTestPrintMs = 0;
};

#endif /* COMPONENTS_FLIGHTTASK_HPP_ */

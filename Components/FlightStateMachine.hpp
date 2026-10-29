/*
 * FlightStateMachine.hpp
 *
 *  Created on: Oct 23, 2026
 *      Author: jaddina
 *
 *  GROUND / FLIGHT decision logic. No HAL or RTOS calls in here, FlightTask
 *  feeds it samples and acts on state changes.
 */
#ifndef COMPONENTS_FLIGHTSTATEMACHINE_HPP_
#define COMPONENTS_FLIGHTSTATEMACHINE_HPP_

#include <stdint.h>

enum class FlightState : uint8_t { GROUND = 0, FLIGHT = 1 };

struct FlightThresholds {
	uint32_t launchAccelMg;        // launch when total accel >= this...
	uint32_t launchAccelWindowMs;  // ...for this long
	float    backupAltM;           // backup launch when altitude above pad > this...
	uint32_t backupAltWindowMs;    // ...for this long
	uint32_t minFlightMs;          // no stillness check before this long in FLIGHT
	uint32_t stillWindowMs;        // still for this long -> back to GROUND
	float    stillAltBandM;        // still if altitude max - min over window <= 2 * band
	uint32_t stillAccelTolMg;      // still if |accel - 1000 mg| <= tol
	uint32_t baroAvgWindowMs;      // baro pressure averaging time constant
	uint32_t padSettleMs;          // backup trigger disabled until the pad reference is this old
	float    padRefTauS;           // pad reference EMA time constant (GROUND only)

	static FlightThresholds Normal();
	static FlightThresholds BenchTest();  // easy to trigger by shaking the board
};

class FlightStateMachine {
public:
	explicit FlightStateMachine(const FlightThresholds& t = FlightThresholds::Normal());

	void SetThresholds(const FlightThresholds& t);
	const FlightThresholds& GetThresholds() const { return th_; }

	// Each returns true when this sample changed the state
	bool OnAccel(int16_t xMg, int16_t yMg, int16_t zMg, uint32_t nowMs);
	bool OnBaro(uint8_t baroId, uint32_t pressurePa, uint32_t nowMs);

	FlightState GetState() const { return state_; }

	// Altitude prediction filter power loss recovery: start straight in a saved state
	void Restore(FlightState s, uint32_t nowMs) { EnterState(s, nowMs); }
	uint32_t GetStateEnteredMs() const { return stateEnteredMs_; }
	uint32_t GetLastAccelMg() const { return lastAccelMg_; }
	bool HasAltitude() const;
	float GetAltitudeAglM() const;  // averaged altitude above pad, 0 if !HasAltitude()

private:
	static constexpr uint8_t NUM_BAROS = 2;
	static constexpr uint32_t MIN_VALID_PA = 30000;
	static constexpr uint32_t MAX_VALID_PA = 110000;
	static constexpr uint32_t BARO_STALE_MS = 1000;  // baro ignored if no valid sample for this long

	struct BaroTrack {
		bool hasAvg;
		float avgPa;           // short moving average of pressure
		uint32_t lastValidMs;
		bool hasRef;
		float refPa;           // pad reference pressure
		uint32_t refStartMs;   // when the reference was (re)initialised
	};

	bool IsBaroCounted(const BaroTrack& b) const;
	bool IsPadSettled() const;
	bool UpdateStillness(uint32_t nowMs, bool accelSample, bool accelInBand);
	void EnterState(FlightState s, uint32_t nowMs);
	void ResetWindows();

	FlightThresholds th_;
	FlightState state_;
	uint32_t stateEnteredMs_;
	uint32_t lastNowMs_;
	uint32_t lastAccelMg_;

	// launch windows
	bool accelAbove_;
	uint32_t accelAboveSinceMs_;
	bool altAbove_;
	uint32_t altAboveSinceMs_;

	// stillness window
	bool stillActive_;
	uint32_t stillSinceMs_;
	bool stillAltInit_;
	float stillAltMin_;
	float stillAltMax_;

	BaroTrack baros_[NUM_BAROS];
};

#endif /* COMPONENTS_FLIGHTSTATEMACHINE_HPP_ */

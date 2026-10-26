/*
 * FlightStateMachine.cpp
 *
 *  Created on: Oct 23, 2026
 *      Author: jaddina
 */

#include "FlightStateMachine.hpp"
#include <math.h>

FlightThresholds FlightThresholds::Normal()
{
	FlightThresholds t;
	t.launchAccelMg = 3000;
	t.launchAccelWindowMs = 250;
	t.backupAltM = 30.0f;
	t.backupAltWindowMs = 500;
	t.minFlightMs = 30000;
	t.stillWindowMs = 10000;
	t.stillAltBandM = 3.0f;
	t.stillAccelTolMg = 200;
	t.baroAvgWindowMs = 500;
	t.padSettleMs = 5000;
	t.padRefTauS = 30.0f;
	return t;
}

FlightThresholds FlightThresholds::BenchTest()
{
	FlightThresholds t = Normal();
	t.launchAccelMg = 1500;
	t.launchAccelWindowMs = 100;
	t.minFlightMs = 3000;
	t.stillWindowMs = 3000;
	return t;
}

FlightStateMachine::FlightStateMachine(const FlightThresholds& t)
	: th_(t), state_(FlightState::GROUND), stateEnteredMs_(0), lastNowMs_(0), lastAccelMg_(0)
{
	ResetWindows();
	for (uint8_t i = 0; i < NUM_BAROS; ++i) {
		baros_[i] = {};
	}
}

void FlightStateMachine::SetThresholds(const FlightThresholds& t)
{
	th_ = t;
	ResetWindows();
}

bool FlightStateMachine::OnAccel(int16_t xMg, int16_t yMg, int16_t zMg, uint32_t nowMs)
{
	lastNowMs_ = nowMs;

	const float x = xMg, y = yMg, z = zMg;
	lastAccelMg_ = static_cast<uint32_t>(sqrtf(x * x + y * y + z * z));

	if (state_ == FlightState::GROUND) {
		if (lastAccelMg_ < th_.launchAccelMg) {
			accelAbove_ = false;  // any dip restarts the launch window
			return false;
		}
		if (!accelAbove_) {
			accelAbove_ = true;
			accelAboveSinceMs_ = nowMs;
		}
		if (nowMs - accelAboveSinceMs_ >= th_.launchAccelWindowMs) {
			EnterState(FlightState::FLIGHT, nowMs);
			return true;
		}
		return false;
	}

	const uint32_t dev = (lastAccelMg_ > 1000) ? (lastAccelMg_ - 1000) : (1000 - lastAccelMg_);
	return UpdateStillness(nowMs, true, dev <= th_.stillAccelTolMg);
}

bool FlightStateMachine::OnBaro(uint8_t baroId, uint32_t pressurePa, uint32_t nowMs)
{
	lastNowMs_ = nowMs;

	if (baroId >= NUM_BAROS || pressurePa < MIN_VALID_PA || pressurePa > MAX_VALID_PA) {
		return false;
	}

	BaroTrack& b = baros_[baroId];
	const float p = static_cast<float>(pressurePa);

	// Short moving average (EMA with the averaging window as time constant)
	if (!b.hasAvg) {
		b.avgPa = p;
		b.hasAvg = true;
	} else {
		const float dt = (nowMs - b.lastValidMs) / 1000.0f;
		const float tau = th_.baroAvgWindowMs / 1000.0f;
		b.avgPa += (p - b.avgPa) * (dt / (tau + dt));
	}

	// Pad reference: slow EMA, only tracks while on the ground
	if (!b.hasRef) {
		b.refPa = b.avgPa;
		b.refStartMs = nowMs;
		b.hasRef = true;
	} else if (state_ == FlightState::GROUND) {
		const float dt = (nowMs - b.lastValidMs) / 1000.0f;
		b.refPa += (b.avgPa - b.refPa) * (dt / (th_.padRefTauS + dt));
	}
	b.lastValidMs = nowMs;

	if (state_ == FlightState::GROUND) {
		if (!HasAltitude() || !IsPadSettled() || GetAltitudeAglM() <= th_.backupAltM) {
			altAbove_ = false;
			return false;
		}
		if (!altAbove_) {
			altAbove_ = true;
			altAboveSinceMs_ = nowMs;
		}
		if (nowMs - altAboveSinceMs_ >= th_.backupAltWindowMs) {
			EnterState(FlightState::FLIGHT, nowMs);
			return true;
		}
		return false;
	}

	return UpdateStillness(nowMs, false, true);
}

bool FlightStateMachine::HasAltitude() const
{
	for (uint8_t i = 0; i < NUM_BAROS; ++i) {
		if (IsBaroCounted(baros_[i])) {
			return true;
		}
	}
	return false;
}

float FlightStateMachine::GetAltitudeAglM() const
{
	float sum = 0.0f;
	uint8_t n = 0;
	for (uint8_t i = 0; i < NUM_BAROS; ++i) {
		const BaroTrack& b = baros_[i];
		if (IsBaroCounted(b)) {
			// Each baro is referenced to its own pad pressure, so their offset cancels out
			sum += 44330.0f * (1.0f - powf(b.avgPa / b.refPa, 0.1903f));
			++n;
		}
	}
	return (n > 0) ? (sum / n) : 0.0f;
}

bool FlightStateMachine::IsBaroCounted(const BaroTrack& b) const
{
	return b.hasAvg && b.hasRef && (lastNowMs_ - b.lastValidMs <= BARO_STALE_MS);
}

bool FlightStateMachine::IsPadSettled() const
{
	for (uint8_t i = 0; i < NUM_BAROS; ++i) {
		const BaroTrack& b = baros_[i];
		if (IsBaroCounted(b) && (lastNowMs_ - b.refStartMs < th_.padSettleMs)) {
			return false;
		}
	}
	return true;
}

/**
 * @brief Stillness check while in FLIGHT. The window restarts whenever accel leaves
 *        1 g +- tol or the altitude spread exceeds the band. Without baros it uses accel only.
 */
bool FlightStateMachine::UpdateStillness(uint32_t nowMs, bool accelSample, bool accelInBand)
{
	if (nowMs - stateEnteredMs_ < th_.minFlightMs) {
		stillActive_ = false;
		return false;
	}

	const bool haveAlt = HasAltitude();
	const float alt = haveAlt ? GetAltitudeAglM() : 0.0f;

	bool restart = !stillActive_ || (accelSample && !accelInBand);

	if (!restart && haveAlt) {
		if (!stillAltInit_) {
			stillAltMin_ = stillAltMax_ = alt;
			stillAltInit_ = true;
		}
		if (alt < stillAltMin_) stillAltMin_ = alt;
		if (alt > stillAltMax_) stillAltMax_ = alt;
		if (stillAltMax_ - stillAltMin_ > 2.0f * th_.stillAltBandM) {
			restart = true;
		}
	}

	if (restart) {
		stillActive_ = true;
		stillSinceMs_ = nowMs;
		stillAltInit_ = haveAlt;
		stillAltMin_ = stillAltMax_ = alt;
		return false;
	}

	if (nowMs - stillSinceMs_ >= th_.stillWindowMs) {
		EnterState(FlightState::GROUND, nowMs);
		return true;
	}
	return false;
}

void FlightStateMachine::EnterState(FlightState s, uint32_t nowMs)
{
	state_ = s;
	stateEnteredMs_ = nowMs;
	ResetWindows();

	if (s == FlightState::GROUND) {
		// Landing site may not be at pad altitude, restart the references from here
		for (uint8_t i = 0; i < NUM_BAROS; ++i) {
			BaroTrack& b = baros_[i];
			b.hasRef = b.hasAvg;
			b.refPa = b.avgPa;
			b.refStartMs = nowMs;
		}
	}
}

void FlightStateMachine::ResetWindows()
{
	accelAbove_ = false;
	accelAboveSinceMs_ = 0;
	altAbove_ = false;
	altAboveSinceMs_ = 0;
	stillActive_ = false;
	stillSinceMs_ = 0;
	stillAltInit_ = false;
	stillAltMin_ = 0.0f;
	stillAltMax_ = 0.0f;
}

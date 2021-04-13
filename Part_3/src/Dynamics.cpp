#include "inc/Dynamics.hpp"

State::State(void) {

}

DroneState::DroneState(void) {
	keys = { "x", "y", "theta", "xDot", "yDot" };
	vec = { {"x", 0}, {"y", 0}, {"theta", 0}, {"xDot", 0}, {"yDot", 0} };
}

DroneState::DroneState(float x, float y, float theta, float xDot, float yDot) : State() {
	keys = { "x", "y", "theta", "xDot", "yDot" };
	vec = { {"x", x}, {"y", y}, {"theta", theta}, {"xDot", xDot}, {"yDot", yDot} };
}

DroneCargoState::DroneCargoState(void) {
	keys = { "xD", "yD", "thetaD", "xDotD", "yDotD", "xC", "yC", "xDotC", "yDotC", "lRope", "lDotRope" };
	vec = { {"xD", 0}, {"yD", droneDefaultHeight}, {"thetaD", 0}, {"xDotD", 0}, {"yDotD", 0}, {"xC", 0}, {"yC", droneDefaultHeight - lZeroRope}, {"xDotC", 0}, {"yDotC", 0}, {"lRope", 0}, {"lDotRope", 0} };
}

DroneCargoState::DroneCargoState(float xD, float yD, float thetaD, float xDotD, float yDotD, float xC, float yC, float xDotC, float yDotC) : State() {
	if (xD == xC && yD == yC) {
		throw "Incorrect input parameters!";
	}

	keys = { "xD", "yD", "thetaD", "xDotD", "yDotD", "xC", "yC", "xDotC", "yDotC", "lRope", "lDotRope" };
	vec = { {"xD", xD}, {"yD", yD}, {"thetaD", thetaD}, {"xDotD", xDotD}, {"yDotD", yDotD}, {"xC", xC}, {"yC", yC}, {"xDotC", xDotC}, {"yDotC", yDotC}, {"lRope", 0}, {"lDotRope", 0} };
}

State::svec State::getState(void) const {
	return vec;
}

void State::setState(const svec& v) {
	for (auto key : keys) {
		vec.at(key) = v.at(key);
	}
}

State::svec State::sum(const State::svec& v1, const State::svec& v2) const {
	svec result;
	if (v1.size() == v2.size()) {
		for (auto key : keys) {
			result[key] = v1.at(key) + v2.at(key);
		}
		return result;
	}
	return {};
}

State::svec State::mul(const svec& v, float h) const {
	svec result;
	for (auto key : keys) {
		result[key] = v.at(key) * h;
	}
	return result;
}

std::vector<float> State::getValues(const svec& v) const {
	std::vector<float> values;
	for (auto key : keys) {
		values.push_back(v.at(key));
	}
	return values;
}

std::vector<std::string> State::getNames(void) const {
	return keys;
}

void State::reset(void) {
	for (auto key : keys) {
		vec.at(key) = 0;
	}
}

void State::print(const svec& v) {
	for (auto key : keys) {
		std::cout << key << " ";
	}
	std::cout << "\n";
	for (auto key : keys) {
		std::cout << v.at(key) << " ";
	}
	std::cout << "\n";
}

State::svec DroneState::f(const svec& v, const std::vector<float>& u) {
	float drag = cDragDrone * sqrtf(powf(v.at("xDot"), 2) + powf(v.at("yDot"), 2)); 
	float thetaDot = 0.0f;

	if (!((v.at("theta") <= thetaMin && u[1] < 0) || (v.at("theta") >= thetaMax && u[1] > 0))) {
		thetaDot = u[1];
	}

	return {
		{"x", v.at("xDot")},
		{"y", v.at("yDot")},
		{"theta", thetaDot},
		{"xDot", (1 / m) * (-u[0] * sinf(v.at("theta")) - drag * v.at("xDot"))},
		{"yDot", (1 / m) * (u[0] * cosf(v.at("theta")) - drag * v.at("yDot")) - g}
	};
}

State::svec DroneCargoState::f(const svec& v, const std::vector<float>& u) {
	float dragDrone = cDragDrone * sqrtf(powf(v.at("xDotD"), 2) + powf(v.at("yDotD"), 2));
	float dragCargo = cDragCargo * sqrtf(powf(v.at("xDotC"), 2) + powf(v.at("yDotC"), 2));

	float lRope = sqrtf(powf(v.at("xD") - v.at("xC"), 2) + powf(v.at("yD") - v.at("yC"), 2));
	float lDotRope = ((v.at("xD") - v.at("xC")) * (v.at("xDotD") - v.at("xDotC")) + (v.at("yD") - v.at("yC")) * (v.at("yDotD") - v.at("yDotC"))) / lRope;

	float fStarRope = kRope * (lRope - lZeroRope) + dRope * lDotRope;
	float fRope = std::max(fStarRope, 0.0f);
	float fRopeX = (fRope * (v.at("xD") - v.at("xC"))) / lRope;
	float fRopeY = (fRope * (v.at("yD") - v.at("yC"))) / lRope;

	float thetaDot = 0;

	if (!((v.at("thetaD") <= thetaMin && u[1] < 0) || (v.at("thetaD") >= thetaMax && u[1] > 0))) {
		thetaDot = u[1];
	}

	return {
		{"xD", v.at("xDotD")},
		{"yD", v.at("yDotD")},
		{"thetaD", thetaDot},
		{"xDotD", (1 / mD) * (-u[0] * sinf(v.at("thetaD")) - dragDrone * v.at("xDotD") - fRopeX)},
		{"yDotD", (1 / mD) * (u[0] * cosf(v.at("thetaD")) - dragDrone * v.at("yDotD") - fRopeY) - g},
		{"xC", v.at("xDotC")},
		{"yC", v.at("yDotC")},
		{"xDotC", (1 / mC) * (-dragCargo * v.at("xDotC") + fRopeX)},
		{"yDotC", (1 / mC) * (-dragCargo * v.at("yDotC") + fRopeY) - g},
		{"lRope", lRope},
		{"lDotRope", lDotRope}
	};
}

void DroneState::reset(void) {
	vec = { {"x", 0}, {"y", 0}, {"theta", 0}, {"xDot", 0}, {"yDot", 0} };
}

void DroneCargoState::reset(void) {
	vec = { {"xD", 0}, {"yD", droneDefaultHeight}, {"thetaD", 0}, {"xDotD", 0}, {"yDotD", 0}, {"xC", 0}, {"yC", droneDefaultHeight - lZeroRope}, {"xDotC", 0}, {"yDotC", 0}, {"lRope", 0}, {"lDotRope", 0} };
}

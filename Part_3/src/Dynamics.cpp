#include "inc/Dynamics.hpp"

State::State(void) { // Base class default contructor

}

DroneState::DroneState(void) { // Derived class default contructor
	keys = { "x", "y", "theta", "xDot", "yDot" }; // State vector names
	vec = { {"x", 0}, {"y", 0}, {"theta", 0}, {"xDot", 0}, {"yDot", 0} }; // Intializing the state vector
}

DroneState::DroneState(float x, float y, float theta, float xDot, float yDot) : State() { // Derived class contructor
	keys = { "x", "y", "theta", "xDot", "yDot" }; // State vector names
	vec = { {"x", x}, {"y", y}, {"theta", theta}, {"xDot", xDot}, {"yDot", yDot} }; // Intializing the state vector
}

DroneCargoState::DroneCargoState(void) { // Derived class default contructor
	keys = { "xD", "yD", "thetaD", "xDotD", "yDotD", "xC", "yC", "xDotC", "yDotC", "lRope", "lDotRope" }; // State vector names
	vec = { {"xD", 0}, {"yD", droneDefaultHeight}, {"thetaD", 0}, {"xDotD", 0}, {"yDotD", 0}, {"xC", 0}, {"yC", droneDefaultHeight - lZeroRope}, {"xDotC", 0}, {"yDotC", 0}, {"lRope", 0}, {"lDotRope", 0} }; // Intializing the state vector
}

DroneCargoState::DroneCargoState(float xD, float yD, float thetaD, float xDotD, float yDotD, float xC, float yC, float xDotC, float yDotC) : State() { // Derived class contructor
	if (xD == xC && yD == yC) { // Drone cannot be at the same position as cargo
		throw "Incorrect input parameters!";
	}

	keys = { "xD", "yD", "thetaD", "xDotD", "yDotD", "xC", "yC", "xDotC", "yDotC", "lRope", "lDotRope" }; // State vector names
	vec = { {"xD", xD}, {"yD", yD}, {"thetaD", thetaD}, {"xDotD", xDotD}, {"yDotD", yDotD}, {"xC", xC}, {"yC", yC}, {"xDotC", xDotC}, {"yDotC", yDotC}, {"lRope", 0}, {"lDotRope", 0} }; // Intializing the state vector
}

State::svec State::getState(void) const { // State vector getter
	return vec;
}

void State::setState(const svec& v) { // State vector setter
	for (auto key : keys) {
		vec.at(key) = v.at(key);
	}
}

State::svec State::sum(const State::svec& v1, const State::svec& v2) const { // Two state vector addition
	svec result; // Declaring the addition result
	if (v1.size() == v2.size()) { // State vector sizes must be equal
		for (auto key : keys) { // For every state vector entry
			result[key] = v1.at(key) + v2.at(key); // Add state vector values at the correponsing names (keys)
		}
		return result;
	}
	return {};
}

State::svec State::mul(const svec& v, float h) const { // State vector multiplcation with a constant
	svec result;
	for (auto key : keys) { // For every state vector entry
		result[key] = v.at(key) * h; // Multiply every state vector value with the constant
	}
	return result;
}

std::vector<float> State::getValues(const svec& v) const { // Get a vector of state vector values (no keys included)
	std::vector<float> values;
	for (auto key : keys) { // For every state vector entry
		values.push_back(v.at(key)); // Add the values at that entry
	}
	return values;
}

std::vector<std::string> State::getNames(void) const { // Get state vector names (keys)
	return keys;
}

void State::print(const svec& v) { // Print the state vector on the screen
	for (auto key : keys) {
		std::cout << key << " "; // First print the names
	}
	std::cout << "\n";
	for (auto key : keys) {
		std::cout << v.at(key) << " "; // Then print the values
	}
	std::cout << "\n";
}

State::svec DroneState::f(const svec& v, const std::vector<float>& u) { // Computing the next state for Drone
	float drag = cDragDrone * sqrtf(powf(v.at("xDot"), 2) + powf(v.at("yDot"), 2)); // Precomputing a value that further appears twice
	float thetaDot = 0.0f; // Default angle value

	if (!((v.at("theta") <= thetaMin && u[1] < 0) || (v.at("theta") >= thetaMax && u[1] > 0))) { // Check if drone angle does not exceed limits
		thetaDot = u[1];
	}

	return { // Return the state vector according to the Drone dynamics
		{"x", v.at("xDot")},
		{"y", v.at("yDot")},
		{"theta", thetaDot},
		{"xDot", (1 / m) * (-u[0] * sinf(v.at("theta")) - drag * v.at("xDot"))},
		{"yDot", (1 / m) * (u[0] * cosf(v.at("theta")) - drag * v.at("yDot")) - g}
	};
}

State::svec DroneCargoState::f(const svec& v, const std::vector<float>& u) { // Computing the next state for Drone with Cargo
	float dragDrone = cDragDrone * sqrtf(powf(v.at("xDotD"), 2) + powf(v.at("yDotD"), 2)); // Precomputing a value that further appears twice
	float dragCargo = cDragCargo * sqrtf(powf(v.at("xDotC"), 2) + powf(v.at("yDotC"), 2)); // Precomputing a value that further appears twice

	float lRope = sqrtf(powf(v.at("xD") - v.at("xC"), 2) + powf(v.at("yD") - v.at("yC"), 2)); // Rope length calculation
	float lDotRope = ((v.at("xD") - v.at("xC")) * (v.at("xDotD") - v.at("xDotC")) + (v.at("yD") - v.at("yC")) * (v.at("yDotD") - v.at("yDotC"))) / lRope; // Derivative of rope length calculation

	float fStarRope = kRope * (lRope - lZeroRope) + dRope * lDotRope; // Temp computation for rope force
	float fRope = std::max(fStarRope, 0.0f); // Rope force computation
	float fRopeX = (fRope * (v.at("xD") - v.at("xC"))) / lRope; // Rope force in X-direction
	float fRopeY = (fRope * (v.at("yD") - v.at("yC"))) / lRope; // Rope force in Y-direction

	float thetaDot = 0; // Default angle value

	if (!((v.at("thetaD") <= thetaMin && u[1] < 0) || (v.at("thetaD") >= thetaMax && u[1] > 0))) { // Check if drone angle does not exceed limits
		thetaDot = u[1];
	}

	return { // Return the state vector according to the Drone with Cargo dynamics
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

void DroneState::reset(void) { // Reseting the Drone state
	vec = { {"x", 0}, {"y", 0}, {"theta", 0}, {"xDot", 0}, {"yDot", 0} };
}

void DroneCargoState::reset(void) { // Reseting the Drone with Cargo state
	vec = { {"xD", 0}, {"yD", droneDefaultHeight}, {"thetaD", 0}, {"xDotD", 0}, {"yDotD", 0}, {"xC", 0}, {"yC", droneDefaultHeight - lZeroRope}, {"xDotC", 0}, {"yDotC", 0}, {"lRope", 0}, {"lDotRope", 0} };
}

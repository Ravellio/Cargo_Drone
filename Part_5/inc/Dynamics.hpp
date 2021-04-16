#ifndef DYNAMICS_HPP
#define DYNAMICS_HPP

#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>
#include <cmath>

class State { // Base class
private:

protected:
	const float g = 9.81f; // Gravity constant

	typedef std::unordered_map<std::string, float> svec; // Declaring a state vector type
	svec vec; // Declaring a state vector

	std::vector<std::string> keys; // State vector names

public:
	State(void); // Default contructor
	virtual ~State(void) = default; // Virtual destructor

	virtual svec f(const svec& v, const std::vector<float>& u) = 0; // Function that computes the next state
	virtual void reset(void) = 0; // State reset

	svec getState(void) const; // Get the state in as a state vector
	void setState(const svec& v); // Set the state vector
	svec sum(const svec& v1, const svec& v2) const; // Add two state vectors
 	svec mul(const svec& v, float h) const; // Multiply a state vector with a constant
	std::vector<float> getValues(const svec& v) const; // Get the values of the state vector
	std::vector<std::string> getNames(void) const; // Get the names of the state vector
	void print(const svec& v); // Print the state vector on the screen
};

class DroneState : public State { // Derived class
private:
	const float m = 3.0f; // Drone mass
	const float cDragDrone = 0.1f; // Drone drag constant

	const float thetaMin = -0.5f; // Minimum drone rotation angle
	const float thetaMax = 0.5f; // Maximum drone rotation angle

public:
	DroneState(void); // Default contructor
	DroneState(float x, float y, float theta, float xDot, float yDot); // Constructor
	virtual ~DroneState(void) = default; // Virtual destructor

	virtual svec f(const svec& v, const std::vector<float>& u) override; // Function that computes the next state
	virtual void reset(void) override; // State reset
};

class DroneCargoState : public State { // Derived class
private:
	const float mD = 3.0f; // Drone mass
	const float mC = 2.0f; // Cargo mass
	const float cDragDrone = 0.1f; // Drone drag constant
	const float cDragCargo = 0.1f; // Cargo drag constant
	const float kRope = 40000.0f; // Rope stiffness 
	const float dRope = 50.0f; // Rope damping

	const float thetaMin = -0.5f; // Minimum drone rotation angle
	const float thetaMax = 0.5f; // Maximum drone rotation angle

	const float droneDefaultHeight = 2.0f; // Drone default height
	
	float lZeroRope = 1.5f; // Rope length

public:
	DroneCargoState(void); // Default contructor
	DroneCargoState(float xD, float yD, float thetaD, float xDotD, float yDotD, float xC, float yC, float xDotC, float yDotC); // Contructor
	virtual ~DroneCargoState(void) = default; // Virtual desctructor

	virtual svec f(const svec& v, const std::vector<float>& u) override; // Function that computes the next state
	virtual void reset(void) override; // State reset
};

#endif
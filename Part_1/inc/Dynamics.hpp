#ifndef DYNAMICS_HPP
#define DYNAMICS_HPP

#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>
#include <cmath>

class State {
private:

protected:
	const float g = 9.81;

	typedef std::unordered_map<std::string, float> svec;
	svec vec;

	std::vector<std::string> keys;

public:
	State();
	virtual ~State(void) = default;

	virtual svec f(const svec& v, const std::vector<float>& u) = 0;

	svec getState(void) const;
	void setState(const svec& v);
	svec sum(const svec& v1, const svec& v2) const;
	svec mul(const svec& v, float h) const;
	std::vector<float> getValues(const svec& v) const;
	std::vector<std::string> getNames(void) const;
	void reset(void);
	void print(const svec& v);
};

class DroneState : public State {
private:
	const float m = 3;
	const float cDragDrone = 0.1;

	const float thetaMin = -0.5;
	const float thetaMax = 0.5;

public:
	DroneState(void);
	DroneState(float x, float y, float theta, float xDot, float yDot);
	virtual ~DroneState(void) = default;

	virtual svec f(const svec& v, const std::vector<float>& u) override;
};

class DroneCargoState : public State {
private:
	const float mD = 3;
	const float mC = 2;
	const float cDragDrone = 0.1;
	const float cDragCargo = 0.1;
	const float lZeroRope = 1.5;
	const float kRope = 40000;
	const float dRope = 50;

	const float thetaMin = -0.5;
	const float thetaMax = 0.5;

	const float droneDefaultHeight = 5;

public:
	DroneCargoState(void);
	DroneCargoState(float xD, float yD, float thetaD, float xDotD, float yDotD, float xC, float yC, float xDotC, float yDotC);
	virtual ~DroneCargoState(void) = default;

	virtual svec f(const svec& v, const std::vector<float>& u) override;
};

#endif
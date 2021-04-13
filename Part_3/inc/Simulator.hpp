#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <memory>

#include "Dynamics.hpp"

class Simulator {
private:

public:
	Simulator(void); // Default contructor
	void euler(const std::unique_ptr<State>& state, const std::vector<float>& u, float h); // Euler method
	void rungeKutta(const std::unique_ptr<State>& state, const std::vector<float>& u, float h); // Runge Kutta method
};

#endif


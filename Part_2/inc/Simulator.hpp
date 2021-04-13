#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <memory>

#include "Dynamics.hpp"
#include "FileReader.hpp"

class Simulator {
private:
	FileReader reader; // File reader

public:
	Simulator(void); // Default constructor
	Simulator(const std::string& readPath); // Constructor
	void euler(const std::unique_ptr<State>& state, float t, float h); // Euler method
	void rungeKutta(const std::unique_ptr<State>& state, float t, float h); // Runge Kutta method
};

#endif


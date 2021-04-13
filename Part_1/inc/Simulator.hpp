#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <memory>

#include "Dynamics.hpp"
#include "FileWriter.hpp"
#include "FileReader.hpp"

class Simulator {
private:
	std::string readPath; // Reading a csv file (input variables)
	std::string writePath; // Writing to a csv file (dynamics state log)

	FileReader reader; // File reader

public:
	Simulator(const std::string& readPath, const std::string& writePath); // Contructor
	void euler(const std::unique_ptr<State>& state, float tStart, float tEnd, float h); // Euler method
	void rungeKutta(const std::unique_ptr<State>& state, float tStart, float tEnd, float h); // Runge Kutta method
};

#endif


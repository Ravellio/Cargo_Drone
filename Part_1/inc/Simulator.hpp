#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <memory>

#include "Dynamics.hpp"
#include "FileWriter.hpp"
#include "FileReader.hpp"

class Simulator {
private:
	std::string readPath;
	std::string writePath;

	FileReader reader;

public:
	Simulator(const std::string& readPath, const std::string& writePath);
	void euler(const std::unique_ptr<State>& state, float tStart, float tEnd, float h);
	void rungeKutta(const std::unique_ptr<State>& state, float tStart, float tEnd, float h);
};

#endif


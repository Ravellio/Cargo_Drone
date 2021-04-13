#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <memory>

#include "Dynamics.hpp"
#include "FileReader.hpp"

class Simulator {
private:
	FileReader reader;

public:
	Simulator(void);
	Simulator(const std::string& readPath);
	void euler(const std::unique_ptr<State>& state, float t, float h);
	void rungeKutta(const std::unique_ptr<State>& state, float t, float h);
};

#endif


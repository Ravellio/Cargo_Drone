#include "inc/Simulator.hpp"

Simulator::Simulator(const std::string& readPath, const std::string& writePath) { // Contructor
	this->readPath = readPath;
	this->writePath = writePath;

	reader = FileReader(readPath); // Creating a file reader
}

void Simulator::euler(const std::unique_ptr<State>& state, float tStart, float tEnd, float h) { // Euler method
	std::vector<std::vector<float>> data; // Storing the data of dynamics state history

	auto v = state->getState(); // Getting a state vector
	auto headers = state->getNames(); // Getting the state vector names
	headers.insert(headers.begin(), "t"); // Inserting "t" for the first column (time)

	for (float t = tStart; t <= tEnd; t += h) { // For every time frame
		auto values = state->getValues(v); // The state vector values
		values.insert(values.begin(), t); // Insert the time value at the front
		data.push_back(values); // Store in data

		v = state->sum(v, state->mul(state->f(v, reader.get(t)), h)); // Perform Forward-Euler algorithm
	}

	FileWriter::write(writePath, headers, data); // Write the results log to a csv file
}

void Simulator::rungeKutta(const std::unique_ptr<State>& state, float tStart, float tEnd, float h) { // Runge-Kutta method
	std::vector<std::vector<float>> data; // Storing the data of dynamics state history

	auto v = state->getState(); // Getting a state vector
	auto headers = state->getNames(); // Getting the state vector names
	headers.insert(headers.begin(), "t"); // Inserting "t" for the first column (time)

	for (float t = tStart; t <= tEnd; t += h) { // For every time frame
		auto values = state->getValues(v); // The state vector values
		values.insert(values.begin(), t); // Insert the time value at the front
		data.push_back(values); // Store in data

		auto u = reader.get(t); // Get the inputs from the file reader
		auto k1 = state->f(v, u); // Start performing Runge-Kutta algorithm
		auto k2 = state->f(state->sum(v, state->mul(k1, h / 2)), u);
		auto k3 = state->f(state->sum(v, state->mul(k2, h / 2)), u);
		auto k4 = state->f(state->sum(v, state->mul(k3, h)), u);
		auto temp1 = state->sum(k1, state->mul(k2, 2));
		auto temp2 = state->sum(temp1, state->mul(k3, 2));
		auto temp3 = state->sum(temp2, k4); 

		v = state->sum(v, state->mul(temp3, h / 6)); // End performing Runge-Kutta algorithm
	}

	FileWriter::write(writePath, headers, data); // Write the results log to a csv file
}

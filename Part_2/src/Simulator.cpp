#include "inc/Simulator.hpp"

Simulator::Simulator(void) { // Default contructor

}

Simulator::Simulator(const std::string& readPath) { // Constructor
	reader = FileReader(readPath); // Creating a file reader
}

void Simulator::euler(const std::unique_ptr<State>& state, float t, float h) { // Euler method
	auto vOld = state->getState(); // Get the current state
	auto vNew = state->sum(vOld, state->mul(state->f(vOld, reader.get(t)), h)); // Compute the new state
	state->setState(vNew); // Set the state vector to the new state
}

void Simulator::rungeKutta(const std::unique_ptr<State>& state, float t, float h) { // Runge Kutta method
	auto vOld = state->getState(); // Get the current state
	auto u = reader.get(t); // Get the inputs from the reader
	auto k1 = state->f(vOld, u); // Start computing the new state
	auto k2 = state->f(state->sum(vOld, state->mul(k1, h / 2.0f)), u);
	auto k3 = state->f(state->sum(vOld, state->mul(k2, h / 2.0f)), u);
	auto k4 = state->f(state->sum(vOld, state->mul(k3, h)), u);
	auto temp1 = state->sum(k1, state->mul(k2, 2.0f));
	auto temp2 = state->sum(temp1, state->mul(k3, 2.0f));
	auto temp3 = state->sum(temp2, k4);
	auto vNew = state->sum(vOld, state->mul(temp3, h / 6.0f)); // Finish computing the new state

	state->setState(vNew); // Set the state vector to the new state
}

#include "inc/Simulator.hpp"

Simulator::Simulator(const std::string& readPath, const std::string& writePath) {
	this->readPath = readPath;
	this->writePath = writePath;

	reader = FileReader(readPath);
}

void Simulator::euler(const std::unique_ptr<State>& state, float tStart, float tEnd, float h) {
	std::vector<std::vector<float>> data;

	auto v = state->getState();
	auto headers = state->getNames();
	headers.insert(headers.begin(), "t");

	for (float t = tStart; t <= tEnd; t += h) {
		auto values = state->getValues(v);
		values.insert(values.begin(), t);
		data.push_back(values);

		v = state->sum(v, state->mul(state->f(v, reader.get(t)), h));
	}

	FileWriter::write(writePath, headers, data);
}

void Simulator::rungeKutta(const std::unique_ptr<State>& state, float tStart, float tEnd, float h) {
	std::vector<std::vector<float>> data;

	auto v = state->getState();
	auto headers = state->getNames();
	headers.insert(headers.begin(), "t");

	for (float t = tStart; t <= tEnd; t += h) {
		auto values = state->getValues(v);
		values.insert(values.begin(), t);
		data.push_back(values);

		auto u = reader.get(t);
		auto k1 = state->f(v, u);
		auto k2 = state->f(state->sum(v, state->mul(k1, h / 2)), u);
		auto k3 = state->f(state->sum(v, state->mul(k2, h / 2)), u);
		auto k4 = state->f(state->sum(v, state->mul(k3, h)), u);
		auto temp1 = state->sum(k1, state->mul(k2, 2));
		auto temp2 = state->sum(temp1, state->mul(k3, 2));
		auto temp3 = state->sum(temp2, k4);

		v = state->sum(v, state->mul(temp3, h / 6));
	}

	FileWriter::write(writePath, headers, data);
}

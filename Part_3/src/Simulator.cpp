#include "inc/Simulator.hpp"

Simulator::Simulator(void) {

}

void Simulator::euler(const std::unique_ptr<State>& state, const std::vector<float>& u, float h) {
	auto vOld = state->getState();
	auto vNew = state->sum(vOld, state->mul(state->f(vOld, u), h));
	state->setState(vNew);
}

void Simulator::rungeKutta(const std::unique_ptr<State>& state, const std::vector<float>& u, float h) {
	std::vector<std::vector<float>> data;

	auto vOld = state->getState();
	auto k1 = state->f(vOld, u);
	auto k2 = state->f(state->sum(vOld, state->mul(k1, h / 2.0f)), u);
	auto k3 = state->f(state->sum(vOld, state->mul(k2, h / 2.0f)), u);
	auto k4 = state->f(state->sum(vOld, state->mul(k3, h)), u);
	auto temp1 = state->sum(k1, state->mul(k2, 2.0f));
	auto temp2 = state->sum(temp1, state->mul(k3, 2.0f));
	auto temp3 = state->sum(temp2, k4);
	auto vNew = state->sum(vOld, state->mul(temp3, h / 6.0f));

	state->setState(vNew);
}

#ifndef OSCILLATIONS_HPP
#define OSCILLATIONS_HPP

#include <vector>

class Oscillations {
private:
	int windowSize;
	
public:
	Oscillations(int windowSize);
	void fft(const std::vector<float>& buff);
};

#endif
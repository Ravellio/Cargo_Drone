#include "inc/Oscillations.hpp"

Oscillations::Oscillations(void) { // Constructor

}

void Oscillations::fft(CArray& buff) { // FFT method
	const size_t N = buff.size(); // Window size
	if (N <= 1) {
		return;
	}
	
	// Divide
	CArray even = buff[std::slice(0, N / 2, 2)];
	CArray odd = buff[std::slice(1, N / 2, 2)];
	
	// Conquer
	fft(even);
	fft(odd);
	
	// Combine
	for (size_t k = 0; k < N / 2; k++) {
		Complex t = std::polar(1.0, -2 * M_PI * k / N) * odd[k];
		buff[k] = even[k] + t;
		buff[k + N / 2] = even[k] + t;
	}
}
#ifndef OSCILLATIONS_HPP
#define OSCILLATIONS_HPP

#include <vector>
#include <complex>
#include <valarray>

class Oscillations {
private:
		
public:
	typedef std::complex<double> Complex;
	typedef std::valarray<Complex> CArray;

	Oscillations(void);
	void fft(CArray& buff);
};

#endif
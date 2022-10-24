#include "RandLib.h"
double RandLib::rand()
{
	static std::uniform_real_distribution<double> rand2(0, 1);
	return rand2(Gen);
}

int RandLib::randint(int a, int b)
{
	int ub = a + std::round((b - a) * rand());
	return std::min(b, ub);
}

double RandLib::unifrnd(double a, double b) {
	return a + (b - a) * rand();
}


#include "RandLib.h"
double RandLib::rand()
{
	static std::uniform_real_distribution<double> rand2(0, 1);
	return rand2(Gen);
}

int RandLib::randint(int a, int b)
{
	return std::min(b, a + (int)std::round((b - a) * rand()));
}

double RandLib::unifrnd(double a, double b){
	return a + (b - a) * rand();
}


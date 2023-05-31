#pragma once
#include <random>
class RandLib {
private:
	int Seed;
public:
	std::default_random_engine Gen;
	double rand(); //continuous uniform distribution in [0,1]
	int randint(int, int); //discrete uniform distribution in [a,b]
	double unifrnd(double, double); //continuous uniform distribution in [a,b]

	explicit RandLib(int sd):
		Seed(sd)
	{ this->Gen.seed(sd); }
	RandLib() {};

};

#pragma once
#include "Solution.h"
#include <unordered_set>

class Run
{
public:
	Solution init;
	Solution best;
	double elapsed_seconds;
	std::vector<unsigned int> seeds;
	std::vector<std::pair<double, double>> convergence;
	Run() {};
	~Run() {};
};
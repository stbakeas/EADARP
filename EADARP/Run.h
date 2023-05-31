#pragma once
#include "Solution.h"

class Run
{
public:
	Solution init;
	Solution best;
	unsigned int best_iter;
	double elapsed_seconds;
	Run() {};
	~Run()=default;
};
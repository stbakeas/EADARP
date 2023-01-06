#pragma once
#include "Solution.h"

class Run
{
public:
	Solution init;
	Solution best;
	std::unordered_set<Solution,Solution::SolutionHash> archive;
	double elapsed_seconds;
	std::vector<unsigned int> seeds;
	std::vector<std::pair<double, double>> convergence;
	Run() {};
	~Run() {};
	void updateArchive();
	Solution NaturalSelection();
};
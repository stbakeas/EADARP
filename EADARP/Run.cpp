#include "Run.h"
#include <algorithm>

float SharingFunction(Solution s1, Solution s2, double sigma) {
	return std::max(0.0, 1 - s1.distanceFrom(s2) / sigma);
}

float NicheCount(Solution s, std::unordered_set<Solution,Solution::SolutionHash> population) {
	float sum = 1;
	for (Solution sol : population) sum += SharingFunction(s, sol, 200);
	return sum;
}

Solution Run::NaturalSelection()
{
	std::vector<Solution> pop;
	for (Solution s : archive) {
		pop.push_back(s);
	}
	std::sort(pop.begin(), pop.end(), [this](Solution s1, Solution s2) {
			return NicheCount(s1,archive) <  NicheCount(s2,archive);
	});
	return pop.front();
}

void Run::updateArchive() {
	std::vector<Solution> dominated;
	for (Solution s1 : archive) {
		for (Solution s2 : archive) {
			if (s1 != s2) {
				if (s2.dominates(s1)) {
					dominated.push_back(s1);
					break;
				}
			}
		}
	}
	for (Solution d : dominated) archive.erase(d);
}

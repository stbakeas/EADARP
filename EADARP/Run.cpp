#include "Run.h"
#include <algorithm>
#include <unordered_set>

double SharingFunction(Solution s1, Solution s2, double sigma) {
	return std::max(0.0, 1.0 - s1.distanceFrom(s2) / sigma);
}

double NicheCount(Solution s, std::unordered_set<Solution,Solution::SolutionHash> population) {
	double sum = 1;
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

#include <iostream>
#include "Instance.h"
#include "Algorithms.h"
#include "Solution.h"
#include "Route.h"
#include <chrono>
#include <thread>
#include "RandLib.h"
#include <cstdint>
#include <sstream>
#include <string>
#include <utility>
#include <fstream>
#include <cmath>
#include <numeric> 


using namespace std;


typedef std::array<float, 3> Configuration;

std::vector<int> rankTransform(std::vector<double>& arr) {
	std::vector<double> copyArr = arr;
	std::vector<int> rankings(arr.size());
	std::sort(copyArr.begin(), copyArr.end());
	std::unordered_map<double, int> hashMap;
	for (double num : copyArr) {
		if (!hashMap.contains(num)) hashMap[num]=hashMap.size() + 1;
	}
	for (int i = 0; i < arr.size(); i++) {
		rankings[i] = hashMap[arr[i]];
	}
	return rankings;
}

double chi_square_critical_value(double degrees_of_freedom, double confidence_level) {
	double alpha = 1.0 - confidence_level;
	double x = degrees_of_freedom / 2.0;
	double a = x - 1.0 / 3.0;
	double b = 1.0 / (9.0 * a);
	double c = pow(a, 3) * (1.0 - b + sqrt(b * (1.0 - b)));
	c = c * (1.0 / (a * a * a));
	double y = 1.0 - alpha / 2.0;
	double z = log(y);
	z = z / log(1.0 - c);
	z = x * (1.0 - c) * pow(z, 3.0);
	double critical_value = x + z;
	return critical_value;
}

Configuration FRace(std::vector<Configuration> candidates,std::vector<std::pair<int,int>> instances,int initRaces, float confidence, int max_execs,int seed) {
	RandLib randlib(seed);
	std::vector<int> candidateIndices(candidates.size());
	std::iota(candidateIndices.begin(),candidateIndices.end(),0);

	std::vector<std::vector<double>> CostMatrix(candidates.size());
	std::vector<std::vector<double>> Blocks;
	std::vector<std::vector<int>> Ranking;
	std::vector<int> RankingSum(candidates.size());
	Blocks.reserve(max_execs);
	Ranking.reserve(max_execs);

	int race = 1;
	while (candidates.size() > 1 && race < max_execs) {
		//Select random instance
		int instanceIndex = randlib.randint(0, instances.size() - 1);
		std::pair<int,int> randomInstance = instances[instanceIndex];
		inst.loadFromFile("Instances-MDHDARP/a"+to_string(randomInstance.first)+"-"+to_string(randomInstance.second)+"hetIUY.txt", 3);
		
		//For each configuration
		Solution init = algorithms::details::Init1();
		std::vector<double> block;
		Run run;
		for (int i:candidateIndices) {
			run = algorithms::IteratedGreedy
			(init, candidates[i][0], 300, candidates[i][1], candidates[i][2]);
			double performance = run.best.AugmentedTchebycheff(0.0) + 0.5 * run.elapsed_seconds;
			//Append the cost of executing candidate i in randomInstance to CostMatrix
			CostMatrix[i].push_back(performance);
			block.push_back(performance);
		}
		Blocks.push_back(block);
		Ranking.push_back(rankTransform(block));

		for (int i:candidateIndices) RankingSum[i] += Ranking[race - 1][i];
		inst.~Instance();

		
		if (race > initRaces) {
			int n = candidates.size();

			//Calculation of T, which is ÷ statistically distributed with n-1 degrees of freedom
			double sum = 0.0;
			for (int j:candidateIndices) sum += pow(RankingSum[j] - race * (n + 1) / 2, 2);
			
			double sum2 = 0.0;
			for (int l = 0; l < Blocks.size(); l++) {
				for (int j:candidateIndices) {
					sum2 += pow(Ranking[l][j], 2);
				}
			}
			double denominator = sum2 - (race * n * pow(n + 1, 2)) / 4;
			double T_value = ((n - 1) * sum) / denominator ;

			//One configurations performs better than at least another one
			if (T_value > chi_square_critical_value(n - 1, 0.95)) {
				int bestConfigIndex = std::min_element(RankingSum.begin(), RankingSum.end())-RankingSum.begin();
				for (int j : candidateIndices) {
					if (j != bestConfigIndex) {
						//Perform pairwise test
						int numerator = std::abs(RankingSum[bestConfigIndex] - RankingSum[j]);
						double denominator2 = sqrt(
							(2*race*(1-T_value/(race*(n-1)))*denominator)
							/
							((race-1)*(n-1))
						);

						
					}
				}
			}

		}
		race++;
	}
	return candidates[0];
}

void VehicleExclusionImpact() {
	unsigned int initialRequests;
	for (int x = 0; x <= 4; x++) {
		ofstream timeFile("exclude" + to_string(x) + "time.txt");
		ofstream costFile("exclude" + to_string(x) + "cost.txt");
		timeFile << "Instance Time\n";
		costFile << "Instance Cost\n";
		printf("%s%i%s\n", "Excluding ", 100 * x / 4, "% of vehicles...");
		std::pair<int, int> vehicleRange{ 9,16 };
		initialRequests = 108;
		inst.controlParameter = x;
		for (int i = vehicleRange.first; i <= vehicleRange.second; i++) {
			inst.loadFromFile("Instances-MDHDARP/a" + to_string(i) + "-" + to_string(initialRequests) + "hetIUY.txt", 3);
			printf("%s%i%c%i\n", "Instance ", i, '-', initialRequests);
			double avgTime;
			double avgCost;
			for (int j = 0; j < 5; j++) {
				Run run = algorithms::IteratedGreedy(algorithms::details::Init1(), 100, INT_MAX,7,0.1);
				avgTime += run.elapsed_seconds;
				avgCost += run.best.AugmentedTchebycheff(0.0);
			}
			avgTime /= 5;
			avgCost /= 5;
			timeFile << i - 8 << " " << avgTime << "\n";
			costFile << i - 8 << " " << avgCost << "\n";
			initialRequests += 12;
			inst.~Instance();
		}
		timeFile.close();
		costFile.close();
	}
}

int main() {

	inst.loadFromFile("Instances-MDHDARP/a10-100hetIUY.txt", 3);
	inst.controlParameter = 4;
	Solution init = algorithms::details::Init4();
	Run run = algorithms::IteratedGreedy(init, 100, INT_MAX, 7, 0.1);
	run.best.Display(0);
	return EXIT_SUCCESS;
}
